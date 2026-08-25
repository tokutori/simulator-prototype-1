//! Host adapters around the platform-independent flight and FBW cores.

#[allow(
    missing_docs,
    clippy::missing_errors_doc,
    clippy::must_use_candidate,
    clippy::pub_underscore_fields
)] // The schema structs mirror documented JSON fields verbatim.
pub mod config;
pub mod controller;
pub mod session;
