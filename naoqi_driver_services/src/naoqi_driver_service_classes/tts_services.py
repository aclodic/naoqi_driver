from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import Say, SayResponse
from std_srvs.srv import Empty, EmptyResponse
from nao_interaction_msgs.srv import String, StringResponse


class TTSServices(AbstractService):
    def __init__(self, super_ns):
        super(TTSServices, self).__init__(
            proxy_name="ALTextToSpeech",
            ns=super_ns+"/tts",
            topics=["say", "stop_all", "set_language"],
            service_types=[Say, Empty, String])

    def say_callback(self, req):
        self.proxy.say(req.text)
        return SayResponse()

    def stop_all_callback(self, _):
        self.proxy.stopAll()
        return EmptyResponse()

    def set_language_callback(self, req):
        try:
            self.proxy.setLanguage(req.request)
        except Exception as e:
            return StringResponse(str(e))
        else:
            return StringResponse()
