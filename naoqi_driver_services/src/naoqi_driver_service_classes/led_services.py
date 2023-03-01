import rospy
from service_abstractclass import AbstractService
from std_srvs.srv import Empty, EmptyResponse
from nao_interaction_msgs.srv import String, StringResponse, Float, FloatResponse
from nao_interaction_msgs.srv import LedsCreateGroup, LedsCreateGroupResponse
from nao_interaction_msgs.srv import LedsEarSetAngle, LedsEarSetAngleResponse
from nao_interaction_msgs.srv import LedsRotateEyes, LedsRotateEyesResponse
from nao_interaction_msgs.srv import LedsSetIntensity, LedsSetIntensityResponse
from nao_interaction_msgs.srv import LedsFade, LedsFadeResponse
from nao_interaction_msgs.srv import LedsFadeRGB, LedsFadeRGBResponse
from nao_interaction_msgs.srv import LedsList, LedsListResponse


class LedServices(AbstractService):
    def __init__(self, super_ns):
        super(LedServices, self).__init__(
            proxy_name="ALLeds",
            ns=super_ns+"/leds",
            topics=["create_group", "ear_set_angle", "fade", "fade_rgb", "get_intensity", "list_groups", "list_leds", "on", "off", "reset", "random_eyes", "rasta", "rotate_eyes", "set_intensity"],
            service_types=[LedsCreateGroup, LedsEarSetAngle, LedsFade, LedsFadeRGB, String, LedsList, LedsList, String, String, String, Float, Float, LedsRotateEyes, LedsSetIntensity])

    def create_group_callback(self, req):
        self.proxy.createGroup(req.name, req.led_names)
        return LedsCreateGroupResponse()

    def ear_set_angle_callback(self, req):
        self.proxy.earLedsSetAngle(req.degrees, req.duration, req.leave_on_at_end)
        return LedsEarSetAngleResponse()

    def fade_callback(self, req):
        self.proxy.fade(req.name, req.intensity, req.duration)
        return LedsFadeResponse()

    def fade_rgb_callback(self, req):
        if req.color_name != "":
            self.proxy.fadeRGB(req.name, req.color_name, req.duration)
        elif req.rgb > 0:
            self.proxy.fadeRGB(req.name, req.rgb, req.duration)
        else:
            self.proxy.fadeRGB(req.name, req.red, req.green, req.blue, req.duration)
        return LedsFadeRGBResponse()

    def get_intensity_callback(self, req):
        r = str(self.proxy.getIntensity(req.request))
        return StringResponse(r)

    def list_groups_callback(self, req):
        if req.name != "":
            r = self.proxy.listGroup(req.name)
        else:
            r = self.proxy.listGroups()
        return LedsListResponse(r)

    def list_leds_callback(self, req):
        if req.name != "":
            r = self.proxy.listLed(req.name)
        else:
            r = self.proxy.listLeds()
        return LedsListResponse(r)

    def on_callback(self, req):
        self.proxy.on(req.request)
        return StringResponse()

    def off_callback(self, req):
        self.proxy.off(req.request)
        return StringResponse()

    def reset_callback(self, req):
        self.proxy.reset(req.request)
        return StringResponse()

    def random_eyes_callback(self, req):
        self.proxy.randomEyes(req.request)
        return FloatResponse()

    def rasta_callback(self, req):
        self.proxy.rasta(req.request)
        return FloatResponse()

    def rotate_eyes_callback(self, req):
        self.proxy.rotateEyes(req.rgb, req.time_for_rotation, req.total_duration)
        return LedsRotateEyesResponse()

    def set_intensity_callback(self, req):
        self.proxy.setIntensity(req.name, req.intensity)
        return LedsSetIntensityResponse()
