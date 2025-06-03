from pamabot_my_first_action.my_action_client import feedback_callback

class DummyFeedback:
    def __init__(self, seconds):
        self.feedback = type("Feedback", (), {"elapsed_time": type("Time", (), {"sec": seconds})})()

def test_feedback_callback_prints_correct_output(capfd):
    feedback = DummyFeedback(15)
    feedback_callback(feedback)

    out, _ = capfd.readouterr()
    assert "15 segundos" in out

