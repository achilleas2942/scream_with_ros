use gst::glib;

mod screamrx;
#[cfg(not(feature = "screamrx-only"))]
mod screamtx;
#[cfg(all(not(feature = "screamrx-only"), feature = "screamtxbw-enabled"))]
mod screamtxbw;
mod gstlidar;

fn plugin_init(plugin: &gst::Plugin) -> Result<(), glib::BoolError> {
    #[cfg(not(feature = "screamrx-only"))]
    screamtx::register(plugin)?;
    #[cfg(all(not(feature = "screamrx-only"), feature = "screamtxbw-enabled"))]
    screamtxbw::register(plugin)?;
    screamrx::register(plugin)?;
    gstlidar::register(plugin)?;
    Ok(())
}

gst::plugin_define!(
    gstscream,
    "GStreamer SCReAM Plugin with LiDAR support",
    plugin_init,
    "1.0",
    "MIT/X11",
    "gstscream",
    "GStreamer",
    "https://example.com",
    "2025-02-25"
);
