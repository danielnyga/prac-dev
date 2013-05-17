import unittest
import jsonrpclib


class TestJsolait(unittest.TestCase):
    def test_echo(self):
        s = jsonrpclib.ServerProxy("http://127.0.0.1:9999", verbose=0)
        reply = s.echo("foo bar")
        print reply
        self.assert_(reply["result"] == "foo bar")



if __name__=="__main__":
    unittest.main()
