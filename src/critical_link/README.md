# critical_link

Ground PCからminiPCへ`/joy`と`/heartbeat/ground_station`を冗長配送するROS 2
パッケージです。USB Wi-Fi、内蔵Wi-Fi、VPN内InternetはIPv4 unicast UDP、
ESP-NOWは専用ESP32を介したUSB serialとして扱います。

## Semantics

senderはROS messageをESP-NOW v1でも収まる最大238 byteのprotocol v2 frameへ変換し、設定済みの
全経路へ同報します。frameには固定の`source_id`を含め、receiverは`source_id`、session、stream
ごとにCRC・payload・sequenceを検証します。同一sourceの同一sequenceの複製と、後着した古い
sequenceは破棄します。あるsourceの再起動は、他sourceのsessionを退役させません。

船側receiverは、検証済みframeをそのまま`/joy`へ出しません。`source_specs`で認可された
sourceだけを候補にし、最新のdeadman有効commandを選びます。優先度は船側設定だけが保持し、
送信側は変更できません。選択済みsourceは正常な間は保持され、高優先度sourceが明示的に
takeoverを要求した場合、または選択sourceがtimeoutした場合だけ中立commandを挟んで切替えます。

Joy payloadは最大32 axesと64 buttonsです。NaN/Inf、壊れたbutton、過大payloadは拒否
します。heartbeatは空payloadのみ受理し、link probeからGround heartbeatを代理生成する
ことはありません。

## Configuration

UDP sender path:

```text
name|ground_bind_ipv4|vessel_destination_ipv4|port
```

UDP receiver path:

```text
name|vessel_bind_ipv4|port
```

NIC名やdefault routeに依存せず、各socketを経路固有の送信元/受信先IPv4へbindします。
船載スマートフォンはminiPCへUSB tetherし、WireGuard/Tailscale等のVPN内IPv4を4本目の
pathに指定してください。public InternetへUDP portやZenoh 7447を直接公開しません。

[ground_sender.yaml](config/ground_sender.yaml)と
[vessel_receiver.yaml](config/vessel_receiver.yaml)へ実機アドレスと安定した
`/dev/serial/by-id/...`を設定します。

## 複数操船 source の設定

各Ground PCまたは操縦器には重複しない`source_id`を設定します。`source_id`は送信者の
固定識別子であり、優先度ではありません。

```yaml
# Ground primary
source_id: 100
deadman_button: 8       # -1 は互換用の常時有効
takeover_button: 9      # -1 は takeover 要求なし

# Vessel receiver。高い値ほど優先度が高い。
source_specs:
  - "100|primary_ground|100"
  - "110|backup_ground|90"
  - "200|onboard_pendant|200"
```

`source_command_timeout_sec`（既定0.25秒）後はreceiverが空のJoyをpublishするため、既存の
0.5秒 command freshness timeoutを待って推力を継続しません。`/heartbeat/ground_station`は、
認可済みsourceの少なくとも一つが新しいheartbeatを送っている間だけpublishします。

IP経路ではCRCは認証ではありません。高優先度sourceを設定する前に、各UDP pathをVPN等の
認証済み経路にし、送信元IP/portの制限も設定してください。

## ROS integration

Ground側ではcanonical topicを直接ネットワークへ出さず、publisherを次へremapします。

```text
joy_node:                         /joy -> /critical_link/input/joy
ground_station_heartbeat_node:   /heartbeat/ground_station
                                  -> /critical_link/input/heartbeat
```

miniPCではreceiverだけが`/joy`と`/heartbeat/ground_station`をpublishします。
既存の`joy_converter`、0.5秒command freshness、規定の60秒Ground heartbeat timeoutは
変更しません。

```bash
ros2 launch critical_link ground_sender.launch.py
ros2 launch critical_link vessel_receiver.launch.py
```

実機へ切り替える前に`test_bringup`のloopback launchで確認できます。

## ESP-NOW

[firmware/critical_link_espnow](firmware/critical_link_espnow/README.md)にGround/Vessel共通の
Arduino sketchがあります。Ground側は同じstreamの新しいframeが来ると古い再送を中止し、
船側gatewayは許可peerのMACとprotocolの`source_id`を照合して、serial出力後に受信元peerへ
application ACKを返します。PMK/LMKを設定したencrypted unicastのみを使用します。
