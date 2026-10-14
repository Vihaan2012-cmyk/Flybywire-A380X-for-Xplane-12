use std::cell::{Cell, RefCell};
use std::sync::atomic::{AtomicU64, Ordering};

use windows::core::{Interface, HSTRING};
use windows::Win32::Foundation::{CloseHandle, HANDLE, HMODULE};
use windows::Win32::Graphics::Direct3D::D3D_DRIVER_TYPE_UNKNOWN;
use windows::Win32::Graphics::Direct3D11::{
    D3D11CreateDevice, ID3D11Device, ID3D11Device1, ID3D11DeviceContext, ID3D11Texture2D, D3D11_BIND_RENDER_TARGET, D3D11_BIND_SHADER_RESOURCE, D3D11_BOX,
    D3D11_CPU_ACCESS_READ, D3D11_CREATE_DEVICE_BGRA_SUPPORT, D3D11_MAPPED_SUBRESOURCE, D3D11_MAP_READ, D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX,
    D3D11_RESOURCE_MISC_SHARED_NTHANDLE, D3D11_SDK_VERSION, D3D11_TEXTURE2D_DESC, D3D11_USAGE_DEFAULT, D3D11_USAGE_STAGING,
};
use windows::Win32::Graphics::Dxgi::Common::{DXGI_FORMAT_B8G8R8A8_UNORM, DXGI_SAMPLE_DESC};
use windows::Win32::Graphics::Dxgi::{CreateDXGIFactory1, IDXGIAdapter, IDXGIFactory1, IDXGIKeyedMutex, IDXGIResource1, DXGI_SHARED_RESOURCE_READ, DXGI_SHARED_RESOURCE_WRITE};

struct Gpu {
    device: ID3D11Device1,
    context: ID3D11DeviceContext,
}

thread_local! {
    static GPU: RefCell<Option<Gpu>> = const { RefCell::new(None) };
    static UNOPENABLE: Cell<u32> = const { Cell::new(0) };
}

static NEXT_GENERATION: AtomicU64 = AtomicU64::new(0);

fn devices() -> Vec<Gpu> {
    let mut out = Vec::new();
    let Ok(factory) = (unsafe { CreateDXGIFactory1::<IDXGIFactory1>() }) else { return out };
    for i in 0.. {
        let Ok(adapter) = (unsafe { factory.EnumAdapters1(i) }) else { break };
        let Ok(adapter) = adapter.cast::<IDXGIAdapter>() else { continue };
        let mut device: Option<ID3D11Device> = None;
        let mut context: Option<ID3D11DeviceContext> = None;
        let made = unsafe {
            D3D11CreateDevice(
                &adapter,
                D3D_DRIVER_TYPE_UNKNOWN,
                HMODULE::default(),
                D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                None,
                D3D11_SDK_VERSION,
                Some(&mut device),
                None,
                Some(&mut context),
            )
        };
        if made.is_err() {
            continue;
        }
        if let (Some(device), Some(context)) = (device.and_then(|d| d.cast::<ID3D11Device1>().ok()), context) {
            out.push(Gpu { device, context });
        }
    }
    out
}

fn with_gpu<R>(f: impl FnOnce(&Gpu) -> R) -> Option<R> {
    GPU.with(|g| g.borrow().as_ref().map(f))
}

pub fn open(handle: HANDLE) -> Option<ID3D11Texture2D> {
    GPU.with(|g| {
        let mut g = g.borrow_mut();
        if let Some(gpu) = g.as_ref() {
            if let Ok(t) = unsafe { gpu.device.OpenSharedResource1::<ID3D11Texture2D>(handle) } {
                return Some(t);
            }
        }
        if UNOPENABLE.with(|u| u.get()) > 3 {
            return None;
        }
        for gpu in devices() {
            if let Ok(t) = unsafe { gpu.device.OpenSharedResource1::<ID3D11Texture2D>(handle) } {
                *g = Some(gpu);
                return Some(t);
            }
        }
        UNOPENABLE.with(|u| u.set(u.get() + 1));
        crate::logging::log("gpu_share: no Direct3D 11 adapter can open Chromium's shared texture");
        None
    })
}

pub fn size(texture: &ID3D11Texture2D) -> (u32, u32) {
    let mut desc = D3D11_TEXTURE2D_DESC::default();
    unsafe { texture.GetDesc(&mut desc) };
    (desc.Width, desc.Height)
}

fn texture(width: u32, height: u32, misc: u32, staging: bool) -> Option<ID3D11Texture2D> {
    let desc = D3D11_TEXTURE2D_DESC {
        Width: width,
        Height: height,
        MipLevels: 1,
        ArraySize: 1,
        Format: DXGI_FORMAT_B8G8R8A8_UNORM,
        SampleDesc: DXGI_SAMPLE_DESC { Count: 1, Quality: 0 },
        Usage: if staging { D3D11_USAGE_STAGING } else { D3D11_USAGE_DEFAULT },
        BindFlags: if staging { 0 } else { (D3D11_BIND_SHADER_RESOURCE.0 | D3D11_BIND_RENDER_TARGET.0) as u32 },
        CPUAccessFlags: if staging { D3D11_CPU_ACCESS_READ.0 as u32 } else { 0 },
        MiscFlags: misc,
    };
    with_gpu(|gpu| {
        let mut out = None;
        unsafe { gpu.device.CreateTexture2D(&desc, None, Some(&mut out)) }.ok()?;
        out
    })
    .flatten()
}

pub fn duplicate(src: &ID3D11Texture2D) -> Option<ID3D11Texture2D> {
    let (w, h) = size(src);
    let copy = texture(w, h, 0, false)?;
    with_gpu(|gpu| unsafe { gpu.context.CopyResource(&copy, src) })?;
    Some(copy)
}

pub struct SharedOut {
    tex: ID3D11Texture2D,
    mutex: IDXGIKeyedMutex,
    handle: HANDLE,
    pub width: u32,
    pub height: u32,
    pub generation: u64,
}

impl Drop for SharedOut {
    fn drop(&mut self) {
        unsafe {
            let _ = CloseHandle(self.handle);
        }
    }
}

impl SharedOut {
    pub fn create(name_for: impl Fn(u64) -> String, width: u32, height: u32) -> Option<Self> {
        let generation = NEXT_GENERATION.fetch_add(1, Ordering::Relaxed) + 1 + std::process::id() as u64 * 1_000_000;
        let tex = texture(width, height, (D3D11_RESOURCE_MISC_SHARED_NTHANDLE.0 | D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX.0) as u32, false)?;
        let resource = tex.cast::<IDXGIResource1>().ok()?;
        let name = HSTRING::from(name_for(generation));
        let handle = unsafe { resource.CreateSharedHandle(None, DXGI_SHARED_RESOURCE_READ.0 | DXGI_SHARED_RESOURCE_WRITE.0, &name) }.ok()?;
        let mutex = tex.cast::<IDXGIKeyedMutex>().ok()?;
        Some(Self { tex, mutex, handle, width, height, generation })
    }

    pub fn copy(&self, src: Option<&ID3D11Texture2D>, popup: Option<(&ID3D11Texture2D, i32, i32)>) -> bool {
        let acquired = unsafe { (Interface::vtable(&self.mutex).AcquireSync)(Interface::as_raw(&self.mutex), 0, 4) };
        if acquired.0 != 0 {
            return false;
        }
        with_gpu(|gpu| unsafe {
            if let Some(src) = src {
                gpu.context.CopyResource(&self.tex, src);
            }
            if let Some((p, x, y)) = popup {
                let (pw, ph) = size(p);
                let (x0, y0) = (x.max(0), y.max(0));
                let right = (x + pw as i32).min(self.width as i32);
                let bottom = (y + ph as i32).min(self.height as i32);
                if right > x0 && bottom > y0 {
                    let source = D3D11_BOX { left: (x0 - x) as u32, top: (y0 - y) as u32, front: 0, right: (right - x) as u32, bottom: (bottom - y) as u32, back: 1 };
                    gpu.context.CopySubresourceRegion(&self.tex, 0, x0 as u32, y0 as u32, 0, p, 0, Some(&source));
                }
            }
        });
        let _ = unsafe { self.mutex.ReleaseSync(0) };
        true
    }
}

pub fn readback(src: &ID3D11Texture2D, staging: &mut Option<ID3D11Texture2D>, dst: &mut [u8]) -> bool {
    let (w, h) = size(src);
    if dst.len() < w as usize * h as usize * 4 {
        return false;
    }
    if staging.as_ref().is_none_or(|s| size(s) != (w, h)) {
        *staging = texture(w, h, 0, true);
    }
    let Some(stage) = staging.as_ref() else { return false };
    with_gpu(|gpu| unsafe {
        gpu.context.CopyResource(stage, src);
        let mut mapped = D3D11_MAPPED_SUBRESOURCE::default();
        if gpu.context.Map(stage, 0, D3D11_MAP_READ, 0, Some(&mut mapped)).is_err() {
            return false;
        }
        let row = w as usize * 4;
        for y in 0..h as usize {
            let s = std::slice::from_raw_parts((mapped.pData as *const u8).add(y * mapped.RowPitch as usize), row);
            dst[y * row..(y + 1) * row].copy_from_slice(s);
        }
        gpu.context.Unmap(stage, 0);
        true
    })
    .unwrap_or(false)
}
