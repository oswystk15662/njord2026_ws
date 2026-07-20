import threading

from yolo.main import _LatestFrameBuffer


def test_latest_frame_replaces_stale_frame():
    frames = _LatestFrameBuffer()
    first = object()
    latest = object()

    assert frames.put(first)
    assert frames.put(latest)
    assert frames.take() is latest


def test_stop_wakes_waiting_consumer_and_rejects_new_frames():
    frames = _LatestFrameBuffer()
    result = []
    consumer = threading.Thread(target=lambda: result.append(frames.take()))
    consumer.start()

    frames.stop()
    consumer.join(timeout=1.0)

    assert not consumer.is_alive()
    assert result == [None]
    assert not frames.put(object())
