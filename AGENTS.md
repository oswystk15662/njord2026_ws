# Repository agent rules

## GitHub write boundary

GitHub上で状態を変更する操作は、対象リポジトリのownerが次のいずれかである場合にのみ許可する。

- `IBO-ASV`
- `oswystk15662`
- `KeioRoboticsAssociation`

この制限は、push、Pull Requestの作成・更新・close・レビュー、Issueの作成・更新・close、コメント、release、tag、workflow dispatch、repository設定変更など、GitHub上の全write操作に適用する。

特に、許可ownerのforkからであっても、別ownerのupstreamリポジトリへPull Request、Issue、コメント、レビューを送ってはならない。外部リポジトリのforkを許可owner配下に作成し、そのforkへpushすることは許可する。

操作前にremote名を信用せず、remote URLまたはGitHub APIで実際のownerを確認する。ownerが不明、または許可リスト外の場合は操作を停止し、ローカル変更、patch、または許可owner配下のfork／リポジトリへのpushに留める。

clone、fetch、閲覧、検索など、外部リポジトリに変更を加えないread-only操作は許可する。
