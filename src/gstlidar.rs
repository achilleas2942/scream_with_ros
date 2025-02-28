use gst::glib;
use gst_base::BaseTransform;
use gst_base::subclass::prelude::*;
use gst_base::subclass::BaseTransformMode;
use gst::subclass::prelude::*;
use gst::prelude::*;

glib::wrapper! {
    pub struct LidarTransform(ObjectSubclass<imp::LidarTransform>) 
        @extends BaseTransform, gst::Element, gst::Object;
}

mod imp {
    use super::*;
    use gst_base::subclass::base_transform::BaseTransformImpl;

    #[derive(Default)]
    pub struct LidarTransform;

    #[glib::object_subclass]
    impl ObjectSubclass for LidarTransform {
        const NAME: &'static str = "LidarTransform";
        type Type = super::LidarTransform;
        type ParentType = BaseTransform;
    }

    impl ObjectImpl for LidarTransform {}
    impl GstObjectImpl for LidarTransform {}
    impl ElementImpl for LidarTransform {}
    impl BaseTransformImpl for LidarTransform {
        const MODE: BaseTransformMode = BaseTransformMode::AlwaysInPlace;
        const PASSTHROUGH_ON_SAME_CAPS: bool = false;
        const TRANSFORM_IP_ON_PASSTHROUGH: bool = false;
    }
}

pub fn register(plugin: &gst::Plugin) -> Result<(), glib::BoolError> {
    gst::Element::register(Some(plugin), "lidartransform", gst::Rank::NONE, LidarTransform::static_type())
}
