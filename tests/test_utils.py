import macposts.utils
import sys


def test_silence(capfd):
    msg1 = "hallo"
    msg2 = "hello"
    msg3 = "ciao"
    stdout = sys.__stdout__
    stderr = sys.__stderr__

    def _write(f, msg):
        f.write(msg)
        f.flush()

    # Normal case
    with macposts.utils.silence():
        _write(stdout, msg1)
        _write(stderr, msg2)
    out, err = capfd.readouterr()
    assert out == ""
    assert err == msg2

    # Specify argument
    with macposts.utils.silence(stdout.fileno()):
        _write(stdout, msg1)
        _write(stderr, msg2)
    out, err = capfd.readouterr()
    assert out == ""
    assert err == msg2

    # Suppress stderr
    with macposts.utils.silence(stderr.fileno()):
        _write(stdout, msg1)
        _write(stderr, msg2)
    out, err = capfd.readouterr()
    assert out == msg1
    assert err == ""

    # No effects outside the context
    _write(stdout, msg1)
    with macposts.utils.silence():
        _write(stdout, msg2)
    _write(stdout, msg3)
    out, err = capfd.readouterr()
    assert out == msg1 + msg3
    assert err == ""

    # Nested
    with macposts.utils.silence():
        _write(stdout, msg1)
        with macposts.utils.silence(stdout.fileno()):
            _write(stdout, msg2)
            with macposts.utils.silence(stderr.fileno()):
                _write(stderr, msg3)
    out, err = capfd.readouterr()
    assert out == ""
    assert err == ""

    # Invalid file descriptor
    with macposts.utils.silence(-1):
        _write(stdout, msg1)
        _write(stderr, msg2)
    out, err = capfd.readouterr()
    assert out == msg1
    assert err == msg2
