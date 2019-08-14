from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import Say, SayResponse
from std_srvs.srv import Empty, EmptyResponse


class TTSServices(AbstractService):
    def __init__(self, super_ns):
        super(TTSServices, self).__init__(
            proxy_name="ALTextToSpeech",
            ns=super_ns+"/tts",
            topics=["say", "stop_all"],
            service_types=[Say, Empty])

    def say_callback(self, req):
        self.proxy.say(req.text)
        return SayResponse()

    def stop_all_callback(self, _):
        self.proxy.stopAll()
        return EmptyResponse()
