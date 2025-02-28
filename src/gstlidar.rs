use gst::glib;
use gst_base::subclass::base_transform::{BaseTransform, BaseTransformMode};
use gst_base::subclass::prelude::*;  
use gst::subclass::prelude::*;        
use gst::prelude::*;

glib::wrapper! {
    pub struct LidarTransform(ObjectSubclass<imp::LidarTransform>) 
        @extends gst_base::BaseTransform, gst::Element, gst::Object;
}

mod imp {
    use super::*;
    use gst_base::subclass::base_transform::BaseTransformImpl;

    #[derive(Default)]
    pub struct LidarTransform;

    #[glib::object_subclass]
    impl ObjectSubclass for LidarTransform {
        const NAME: &'static str = "LidarTransform";
        type ParentType = BaseTransform;
    }

    impl ObjectImpl for LidarTransform {}
    impl ElementImpl for LidarTransform {}
    impl BaseTransformImpl for LidarTransform {}
}

pub fn register(plugin: &gst::Plugin) -> Result<(), glib::BoolError> {
    LidarTransform::register(plugin)
}
