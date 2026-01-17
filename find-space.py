import os
import shutil
from pathlib import Path

def get_size(path):
    """Calculate size of file or directory in bytes."""
    if os.path.isfile(path):
        return os.path.getsize(path)
    total_size = 0
    try:
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                # skip if it is symbolic link
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)
    except (PermissionError, FileNotFoundError):
        pass
    return total_size

def format_bytes(size):
    """Convert bytes to a human-readable string."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size < 1024:
            return f"{size:.2f} {unit}"
        size /= 1024

def scan_folders(start_path, limit=10):
    print(f"\n🔍 Scanning: {start_path}")
    items = []
    
    try:
        for entry in os.scandir(start_path):
            size = get_size(entry.path)
            items.append((entry.name, size))
    except PermissionError:
        print("⚠️ Run as sudo to scan system directories.")
        return

    # Sort by size descending
    items.sort(key=lambda x: x[1], reverse=True)

    print(f"{'Folder/File':<30} | {'Size':<10}")
    print("-" * 45)
    for name, size in items[:limit]:
        print(f"{name[:30]:<30} | {format_bytes(size):<10}")

if __name__ == "__main__":
    # 1. Check Root Partition
    total, used, free = shutil.disk_usage("/")
    print(f"📊 Disk Summary (Root /):")
    print(f"   Total: {format_bytes(total)}")
    print(f"   Used:  {format_bytes(used)} ({used/total:.1%})")
    print(f"   Free:  {format_bytes(free)}")

    # 2. Check Home Directory (Personal Bloat)
    scan_folders(os.path.expanduser("~"))

    # 3. Check Workspace (Project Bloat)
    workspace_path = os.getcwd()
    scan_folders(workspace_path)

    # 4. Critical System Paths (Requires Sudo often)
    print("\n💡 Tip: If Home and Workspace look small, check '/var/lib/docker'")
    print("   Run: sudo du -sh /var/lib/docker")
