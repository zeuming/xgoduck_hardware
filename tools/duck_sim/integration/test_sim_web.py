import io, unittest
from unittest.mock import Mock, patch
import sim_web

class GatewayTests(unittest.TestCase):
 def fixture(self,path='/sim/api/simulate'):
  handler=Mock();handler.path=path;handler.headers={'Cookie':'session','X-CSRF-Token':'good'}
  handler.framing.return_value=2;handler.rfile=io.BytesIO(b'{}')
  frontend=Mock();frontend.authenticated.return_value=True;frontend.token='good';frontend.authentication_required=True
  web=Mock();return handler,frontend,web
 def test_non_sim_route_is_untouched(self):
  h,f,w=self.fixture('/api/motion');self.assertFalse(sim_web.route(h,f,w,post=True));h.framing.assert_not_called()
 def test_wrong_token_never_reaches_backend(self):
  h,f,w=self.fixture();h.headers['X-CSRF-Token']='wrong'
  with patch.object(sim_web.http.client,'HTTPConnection') as c:sim_web.route(h,f,w,post=True);c.assert_not_called()
  self.assertEqual(h.reply.call_args.args[0],403)
 def test_unknown_operation_never_reaches_backend(self):
  h,f,w=self.fixture('/sim/api/motion')
  with patch.object(sim_web.http.client,'HTTPConnection') as c:sim_web.route(h,f,w,post=True);c.assert_not_called()
  self.assertEqual(h.reply.call_args.args[0],404)
 def test_no_session_never_reaches_backend(self):
  h,f,w=self.fixture();f.authenticated.return_value=False
  with patch.object(sim_web.http.client,'HTTPConnection') as c:sim_web.route(h,f,w,post=True);c.assert_not_called()
  self.assertEqual(h.reply.call_args.args[0],401)
 def test_only_fixed_loopback_destination(self):
  h,f,w=self.fixture()
  with patch.object(sim_web.http.client,'HTTPConnection') as c:
   response=c.return_value.getresponse.return_value;response.read.return_value=b'{}';response.status=200;response.getheader.return_value='application/json'
   sim_web.route(h,f,w,post=True)
   c.assert_called_once_with('127.0.0.1',8788,timeout=45)
   c.return_value.request.assert_called_once_with('POST','/api/simulate',body=b'{}',headers={'Content-Type':'application/json'})
 def test_busy_gateway_fails_without_backend(self):
  h,f,w=self.fixture()
  with patch.object(sim_web,'_slots') as slots,patch.object(sim_web.http.client,'HTTPConnection') as c:
   slots.acquire.return_value=False;sim_web.route(h,f,w,post=True);c.assert_not_called();slots.release.assert_not_called()
  self.assertEqual(h.reply.call_args.args[0],503)
 def test_joint_link_passes_framing_without_changing_request_url(self):
  h,f,w=self.fixture('/sim/?joint=14&v=2')
  def framing(**kwargs):
   self.assertEqual(h.path,'/sim/');return 0
  h.framing.side_effect=framing
  with patch.object(sim_web.http.client,'HTTPConnection') as c:
   response=c.return_value.getresponse.return_value;response.read.return_value=b'<html></html>';response.status=200;response.getheader.return_value='text/html'
   sim_web.route(h,f,w)
   c.return_value.request.assert_called_once_with('GET','/',body=None,headers={})
  self.assertEqual(h.path,'/sim/?joint=14&v=2')
  self.assertEqual(h.reply.call_args.args[0],200)
 def test_bad_query_and_post_query_never_reach_backend(self):
  for path,post in [('/sim/?joint=14&joint=12',False),('/sim/?joint=14&action=start',False),
                    ('/sim/?joint=%31%34',False),('/sim/?joint=14#x',False),
                    ('/sim/?',False),('/sim/?v=1234',False),
                    ('/sim/api/live?joint=14',False),('/sim/app.js?v=2',False),
                    ('/sim/?joint=14',True),('/sim/api/live/connect?v=2',True)]:
   with self.subTest(path=path,post=post):
    h,f,w=self.fixture(path)
    with patch.object(sim_web.http.client,'HTTPConnection') as c:
     sim_web.route(h,f,w,post=post);c.assert_not_called()
    self.assertEqual(h.reply.call_args.args[0],400)
 def test_query_does_not_skip_authentication(self):
  h,f,w=self.fixture('/sim/?joint=14');f.authenticated.return_value=False
  with patch.object(sim_web.http.client,'HTTPConnection') as c:
   sim_web.route(h,f,w);c.assert_not_called()
  self.assertEqual(h.reply.call_args.args[0],401)
 def test_anonymous_joint_link_establishes_document_session(self):
  h,f,w=self.fixture('/sim/?joint=14');f.authentication_required=False;f.anonymous_session.return_value=('token',True)
  with patch.object(sim_web.http.client,'HTTPConnection') as c:
   r=c.return_value.getresponse.return_value;r.read.return_value=b'<html/>';r.status=200;r.getheader.return_value='text/html'
   sim_web.route(h,f,w)
  f.anonymous_session.assert_called_once_with('session');w.session_cookie.assert_called_once_with('token')
  self.assertEqual(h.reply.call_args.args[0],200)
 def test_invalid_framing_is_not_reported_as_backend_outage(self):
  h,f,w=self.fixture('/sim/?joint=14');h.framing.side_effect=ValueError('工作台地址无效')
  with patch.object(sim_web.http.client,'HTTPConnection') as c:
   sim_web.route(h,f,w);c.assert_not_called()
  self.assertEqual(h.path,'/sim/?joint=14');self.assertEqual(h.reply.call_args.args,(400,{'error':'工作台地址无效'}))
 def test_backend_outage_is_still_service_unavailable(self):
  h,f,w=self.fixture('/sim/')
  with patch.object(sim_web.http.client,'HTTPConnection',side_effect=OSError('connection refused')):
   sim_web.route(h,f,w)
  self.assertEqual(h.reply.call_args.args[0],503)

if __name__=='__main__':unittest.main()
