/***********************************************************************************************************************
 * Copyright (c) 2019 by the authors
 *
 * Author: André Borrmann
 * License: Apache License 2.0
 **********************************************************************************************************************/
#![doc(html_root_url = "https://docs.rs/ruspiro-allocator/0.4.1")]
#![cfg_attr(not(any(test, doctest)), no_std)]
#![feature(alloc_error_handler)]
//! # Custom Allocator for HEAP memory allocations
//!
//! This crate provides a custom allocator for heap memory. If any baremetal crate uses functions and structures from
//! the ``alloc`` crate an allocator need to be provided as well. However, this crate does not export any public
//! API to be used. It only encapsulates the memeory allocator that shall be linked into the binary.
//!
//! # Usage
//!
//! To link the custom allocator with your project just add the usage to your main crate rust file like so:
//! ```ignore
//! extern crate ruspiro_allocator;
//! ```
//! Wherever you define the usage of the ``ruspiro-allocator`` crate within your project does not matter. But as soon
//! as this is done the dynamic structures requiring heap memory allocations from the ``alloc`` crate could be used like
//! so:
//! ```
//! #[macro_use]
//! extern crate alloc;
//! use alloc::{boxed::Box, vec::Vec};
//!
//! fn main() {
//!     let mut v: Vec<u32> = vec![10, 20];
//!     let b: Box<u16> = Box::new(10);
//!     v.push(12);
//! }
//! ```
//!

/// this specifies the custom memory allocator to use whenever heap memory need to be allocated or freed
#[cfg_attr(not(any(test, doctest)), global_allocator)]
static ALLOCATOR: RusPiRoAllocator = RusPiRoAllocator;

use core::alloc::{GlobalAlloc, Layout};
use ruspiro_mmu as mmu;

mod memory;

struct RusPiRoAllocator;

unsafe impl GlobalAlloc for RusPiRoAllocator {
    #[inline]
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        memory::alloc(layout.size(), layout.align())
    }

    #[inline]
    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        memory::free(ptr)
    }

    #[inline]
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = memory::alloc(layout.size(), layout.align());
        memset(ptr, 0x0, layout.size());
        ptr
    }
}

#[cfg(not(any(test, doctest)))]
#[alloc_error_handler]
#[allow(clippy::empty_loop)]
fn alloc_error_handler(_: Layout) -> ! {
    // TODO: how to handle memory allocation errors?
    loop {}
}

/// Request the given address to be aligned to the next memory page based on the current architecture.
pub fn page_align(addr: usize) -> usize {
    mmu::page_align(addr)
}

/// Allocate DMA memory with the given size. This will return the virtual memory address as well as the DMA(BUS) memory 
/// address. The memory allocated by this function will be configured within the MMU to be coherent cross all cores and
/// not cached.
///
/// # Safety 
/// This is a raw memory allocation on the HEAP. The raw pointer that will be returned may point to uninitialized memory
/// therefore this function does not adhere to any guaranties typically required by rust. So casting the returned 
/// pointer to a valid structure is the responsibility of the caller that should have requested a memory size that will
/// be able to cover the type the pointer will be casted to.
/// The returned DMA/BUS address should not be used for any assignment except for passing it the VideoCore if it 
/// requires to access the exact same memory location. Care need to be taken if this kind of memory is shared between 
/// ARM and VideoCore as the ARM (Rust) side cannot be aware of any changes happen to the memory on VideoCore side which
/// might easily lead to undefined behavior.
/// Memory allocated with this function has to be freed with the [dealloc_dma] function to ensure that care is taken for
/// maintaining the memory attributes.
pub unsafe fn alloc_dma(size: usize) -> (*mut u8, *mut u8) {
    // alignment does always need to be a full PAGE. The actual page size used is provided
    // in the MMU crate. As allocation of DMA memory requires specific memory attributes to be set for the MMU 
    // translation table which can only be done on a page level the requested memory size should always fit into a whole
    // page to ensure subsequent memory allocation are not done within the configured page when they need normal memory
    // attributes
    #[cfg(target_arch = "aarch64")]
    let (virt_addr, size) = {
        let pages = (size >> mmu::config::PAGE_SHIFT) + 1;
        (memory::alloc_page(pages, mmu::config::PAGE_SIZE), pages * mmu::config::PAGE_SIZE)
    };
    #[cfg(target_arch = "arm")]
    let (virt_addr, size) = {
        let sections = ((size >> mmu::config::SECTION_SHIFT) + 1);
        (memory::alloc_page(sections, mmu::config::SECTION_SIZE), sections * mmu::config::SECTION_SIZE)
    };
    
    memset(virt_addr, 0x0, size);
    // once we have the memory allocated starting at a page address we can maintain the MMU page attributes
    #[cfg(target_arch = "aarch64")]
    {
        // from the virtual address we can directly derive the page Id - which will be an index into the mmu page tables
        let page_id = (virt_addr as usize) >> mmu::config::PAGE_SHIFT;
        // we can also derive how many pages we need to configure the same way
        let page_num = size >> mmu::config::PAGE_SHIFT;
        // now we cen request the MMU to maintain those pages with the desired memory attributes
        mmu::maintain_pages(
            virt_addr,
            page_id, page_num, 
            0
            | 0b1 << 54 // UXN User never execute
            | 0b1 << 53 // PXN priviliged bit
            | 0b1 << 51 // DBM dirty bit management flag
            | 0b1 << 10 // AF bit
            | 0b11 << 8 // SH shareable flag set to outer shareable
            | 0b000 << 2 // memAttr. index = 0 => coherent memory nGnRnE
        );
    }
    #[cfg(target_arch = "arm")]
    {
        let section_id = (virt_addr as usize) >> mmu::config::SECTION_SHIFT;
        let section_num = (size >> mmu::config::SECTION_SHIFT);
        // now we cen request the MMU to maintain those pages with the desired memory attributes
        mmu::maintain_sections(
            virt_addr,
            section_id, section_num,
            0x10412,
        );
    }

    // assuming we would retrieve the addr from the memory allocation as virtual address we would nedd to map this to 
    // its physical/BUS one, as we currently maintain 1:1 virtual to physical memory mapping both would be the same
    // and the BUS address simply need to be ORed with 0xC000_0000 to omit L2 caching
    let phys_addr = ((virt_addr as usize ) | 0xC000_0000) as *mut u8;

    (virt_addr, phys_addr)
}

/// Release memory previously allocated with [alloc_dma]
/// # Safety
/// This unction should only be called on memory that is previously allocated with alloc_dma as it will ensure the 
/// memory attribute settings for this region will be restored to "standard"
pub unsafe fn dealloc_dma(ptr: *mut u8) {
    memory::free(ptr);
    // TODO: revert the MMU memory configuration to allow this part to be used as "normal" memory.
    // It's not implemented yet, as the only DMA allocation currently implemented is for devices which always live as 
    // long as the whole software/kernel is running, so it practically will never being freed.
}

extern "C" {
    // reference to the compiler built-in function
    fn memset(ptr: *mut u8, value: i32, size: usize) -> *mut u8;
}
