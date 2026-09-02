#!/usr/bin/env python3
"""
Smart Wheelchair QR Code Generator
Generates high-resolution PNG QR codes and renders an ASCII QR code in terminal console
for mobile users to instantly connect to the Tailscale Summoning Portal.
"""

import sys
import os
import argparse
import qrcode

DEFAULT_SUMMON_URL = "https://ducky.tail0de3ff.ts.net/?mode=summon"
DEFAULT_PORTAL_URL = "https://ducky.tail0de3ff.ts.net"

def generate_qr(url, output_file="wheelchair_summon_qr.png", title="Smart Wheelchair Summoning Portal"):
    print("\n" + "═" * 60)
    print(f"  {title}")
    print("═" * 60)
    print(f"  URL: {url}")
    print("─" * 60)
    
    # 1. Print ASCII QR Code to terminal console
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=1,
        border=2,
    )
    qr.add_data(url)
    qr.make(fit=True)
    
    print("\n  Scan QR Code below with your mobile camera:")
    qr.print_ascii(invert=True)
    
    # 2. Save high-resolution PNG image
    img_qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4,
    )
    img_qr.add_data(url)
    img_qr.make(fit=True)
    
    img = img_qr.make_image(fill_color="black", back_color="white")
    
    output_path = os.path.abspath(output_file)
    img.save(output_path)
    print(f"\n  ✓ High-resolution PNG saved to: {output_path}")
    print("═" * 60 + "\n")

def main():
    parser = argparse.ArgumentParser(description="Generate QR codes for Smart Wheelchair Tailscale Frontend")
    parser.add_argument("--url", type=str, default=DEFAULT_SUMMON_URL, help="Custom URL for QR code generation")
    parser.add_argument("--out", type=str, default="wheelchair_summon_qr.png", help="Output PNG file path")
    parser.add_argument("--all", action="store_true", help="Generate QR codes for both Summoning Portal and Main Portal")
    args = parser.parse_args()

    if args.all:
        generate_qr(DEFAULT_SUMMON_URL, "wheelchair_summon_qr.png", "SMART WHEELCHAIR - MOBILE SUMMON PORTAL")
        generate_qr(DEFAULT_PORTAL_URL, "wheelchair_portal_qr.png", "SMART WHEELCHAIR - MAIN PORTAL")
    else:
        generate_qr(args.url, args.out, "SMART WHEELCHAIR FRONTEND ACCESS")

if __name__ == "__main__":
    main()
