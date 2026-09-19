//! Installer and updater for the FlyByWire A380X X-Plane port (GPL-3.0).
//!
//! Asks its questions in Windows dialogs (the licence notice, your X-Plane 12
//! folder, your SASL 3 archive), then on a background thread: fetches the
//! release manifest from the release server, downloads the release, verifies
//! its SHA-256, and runs the release's own install steps (`install.json`
//! inside the archive: SASL from your archive, then the whole aircraft). The
//! installed version is recorded next to the aircraft, so running it again
//! updates in place.
//!
//! SASL is proprietary and never in a release; everything else is. The
//! steps also support converting from your own MSFS package (`run`,
//! `copy_from_msfs`), for a release that ships without converted content.

#![windows_subsystem = "windows"]

mod install;

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::mpsc::{channel, Receiver};

use native_windows_gui as nwg;

use install::{Answers, Msg};

/// The release server (the Cloudflare Worker's URL), fixed at build time
/// from `FBW_XP_RELEASES_URL` (see distribution/README.md); the same variable
/// at run time overrides it (a local `wrangler dev`, a mirror).
pub const DEFAULT_SERVER: &str = match option_env!("FBW_XP_RELEASES_URL") {
    Some(url) => url,
    None => "http://127.0.0.1:8787",
};

#[derive(Default)]
struct App {
    window: nwg::Window,
    notice: nwg::Notice,
    status: nwg::Label,
    progress: nwg::ProgressBar,
    log: nwg::TextBox,
    close: nwg::Button,
    rx: RefCell<Option<Receiver<Msg>>>,
}

fn main() {
    nwg::init().expect("Windows GUI");
    let _ = nwg::Font::set_global_family("Segoe UI");

    let Some(answers) = ask() else { return };

    let app = Rc::new(RefCell::new(App::default()));
    build(&mut app.borrow_mut()).expect("installer window");
    let (tx, rx) = channel();
    *app.borrow().rx.borrow_mut() = Some(rx);
    let sender = app.borrow().notice.sender();
    std::thread::spawn(move || install::run(answers, tx, sender));

    let a = app.clone();
    let handle = app.borrow().window.handle;
    let handler = nwg::full_bind_event_handler(&handle, move |evt, _data, h| {
        let app = a.borrow();
        match evt {
            nwg::Event::OnNotice => app.drain(),
            nwg::Event::OnButtonClick if h == app.close.handle => nwg::stop_thread_dispatch(),
            nwg::Event::OnWindowClose if h == app.window.handle => nwg::stop_thread_dispatch(),
            _ => {}
        }
    });
    nwg::dispatch_thread_events();
    nwg::unbind_event_handler(&handler);
}

fn build(a: &mut App) -> Result<(), nwg::NwgError> {
    nwg::Window::builder()
        .size((620, 420))
        .position((300, 200))
        .title("FlyByWire A380X for X-Plane 12")
        .build(&mut a.window)?;
    nwg::Notice::builder().parent(&a.window).build(&mut a.notice)?;
    nwg::Label::builder().text("Starting...").position((16, 12)).size((588, 22)).parent(&a.window).build(&mut a.status)?;
    nwg::ProgressBar::builder().position((16, 40)).size((588, 20)).range(0..1000).parent(&a.window).build(&mut a.progress)?;
    nwg::TextBox::builder()
        .position((16, 70))
        .size((588, 290))
        .readonly(true)
        .flags(nwg::TextBoxFlags::VISIBLE | nwg::TextBoxFlags::VSCROLL | nwg::TextBoxFlags::AUTOVSCROLL)
        .parent(&a.window)
        .build(&mut a.log)?;
    nwg::Button::builder().text("Close").position((504, 370)).size((100, 30)).enabled(false).parent(&a.window).build(&mut a.close)?;
    Ok(())
}

impl App {
    fn drain(&self) {
        let rx = self.rx.borrow();
        let Some(rx) = rx.as_ref() else { return };
        while let Ok(msg) = rx.try_recv() {
            match msg {
                Msg::Status(s) => {
                    self.status.set_text(&s);
                    self.line(&s);
                }
                Msg::Log(s) => self.line(&s),
                Msg::Progress(p) => self.progress.set_pos((p.clamp(0.0, 1.0) * 1000.0) as u32),
                Msg::Done(result) => {
                    self.close.set_enabled(true);
                    match result {
                        Ok(summary) => {
                            self.status.set_text("Done.");
                            self.line(&summary);
                            nwg::modal_info_message(&self.window, "Installed", &summary);
                        }
                        Err(e) => {
                            self.status.set_text("Failed.");
                            self.line(&format!("ERROR: {e}"));
                            nwg::modal_error_message(&self.window, "Installation failed", &e);
                        }
                    }
                }
            }
        }
    }

    fn line(&self, s: &str) {
        let mut text = self.log.text();
        text.push_str(s);
        text.push_str("\r\n");
        self.log.set_text(&text);
    }
}

// ---- the questions --------------------------------------------------------

fn yes_no(title: &str, content: &str) -> bool {
    let p = nwg::MessageParams {
        title,
        content,
        buttons: nwg::MessageButtons::YesNo,
        icons: nwg::MessageIcons::Question,
    };
    nwg::message(&p) == nwg::MessageChoice::Yes
}

fn pick(title: &str, folder: bool, filter: Option<&str>) -> Option<PathBuf> {
    let mut dialog = nwg::FileDialog::default();
    let mut b = nwg::FileDialog::builder()
        .title(title)
        .action(if folder { nwg::FileDialogAction::OpenDirectory } else { nwg::FileDialogAction::Open });
    if let Some(f) = filter {
        b = b.filters(f);
    }
    b.build(&mut dialog).ok()?;
    if dialog.run(None::<&nwg::Window>) {
        dialog.get_selected_item().ok().map(|s| PathBuf::from(s.to_string_lossy().into_owned()))
    } else {
        None
    }
}

/// Confirm a detected path, or pick one. `None` if the user cancels.
fn confirm_or_pick(what: &str, detected: Option<PathBuf>, folder: bool, filter: Option<&str>) -> Option<PathBuf> {
    if let Some(d) = detected {
        if yes_no(what, &format!("{what}:\n\n{}\n\nUse this?", d.display())) {
            return Some(d);
        }
    }
    pick(&format!("Choose {what}"), folder, filter)
}

fn ask() -> Option<Answers> {
    let licence = "FlyByWire A380X for X-Plane 12\n\n\
        Free software under the GNU GPL v3. FlyByWire's original 3D models are \
        licensed CC BY-NC 4.0 (non-commercial, with attribution to FlyByWire \
        Simulations). Not affiliated with or endorsed by Microsoft, Laminar Research \
        or FlyByWire Simulations.\n\n\
        Microsoft Flight Simulator (c) Microsoft Corporation; FlyByWire's aircraft was \
        created under Microsoft's Game Content Usage Rules. Liveries remain their authors'. \
        SASL is not included: you will be asked for your own copy.\n\nContinue?";
    if !yes_no("FlyByWire A380X for X-Plane 12", licence) {
        return None;
    }
    let xplane = confirm_or_pick("your X-Plane 12 folder", install::detect_xplane(), true, None)?;
    let sasl = confirm_or_pick("your SASL 3 archive (SASL is proprietary and never downloaded)", None, false, Some("Zip(*.zip)"))?;
    let server = std::env::var("FBW_XP_RELEASES_URL").unwrap_or_else(|_| DEFAULT_SERVER.to_owned());
    Some(Answers { xplane, msfs_package: install::detect_fbw_package(), livery: None, sasl, server })
}
