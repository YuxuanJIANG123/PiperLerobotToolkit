from lerobot.common.robot_devices.teleop.terminal_keyboard import _TerminalKeyboardService


def test_terminal_parser_decodes_printable_keys_and_arrows():
    service = _TerminalKeyboardService()
    received = []
    service._callbacks.add(received.append)
    remaining = service._parse(b"qQ\x1b[C\x1b[D", flush_escape=True)
    assert remaining == b""
    assert received == ["q", "q", "right", "left"]


def test_terminal_parser_waits_before_treating_escape_as_escape_key():
    service = _TerminalKeyboardService()
    received = []
    service._callbacks.add(received.append)
    remaining = service._parse(b"\x1b", flush_escape=False)
    assert remaining == b"\x1b"
    assert received == []
    assert service._parse(remaining, flush_escape=True) == b""
    assert received == ["esc"]
