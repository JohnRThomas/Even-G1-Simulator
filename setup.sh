#!/bin/sh
# Download Renode into .renode/ next to this script, where g1sim.py looks for
# it. Linux x86_64 only; elsewhere install Renode 1.16+ yourself and put it on
# PATH or in $RENODE. Run again to reinstall.
#
# Needs curl, sha256sum and tar.
set -eu

VERSION=1.16.1
URL="https://github.com/renode/renode/releases/download/v$VERSION/renode-$VERSION.linux-portable-dotnet.tar.gz"
SHA256=00e113cdbd0f5354cf2f64bbe3f5a070d8958409542fca66e45ac97d982938c0

cd "$(dirname "$0")"

if [ "$(uname -s) $(uname -m)" != "Linux x86_64" ]; then
    echo "setup.sh: no pinned Renode build for $(uname -s) $(uname -m)." >&2
    echo "Install Renode $VERSION or newer from https://renode.io and set \$RENODE." >&2
    exit 1
fi

tmp=$(mktemp -d .renode-download.XXXXXX)
trap 'rm -rf "$tmp"' EXIT

echo "setup.sh: downloading $URL"
curl --fail --location --progress-bar --output "$tmp/renode.tar.gz" "$URL"

if ! echo "$SHA256  $tmp/renode.tar.gz" | sha256sum --check --status -; then
    echo "setup.sh: checksum mismatch, expected $SHA256, got:" >&2
    sha256sum "$tmp/renode.tar.gz" >&2
    exit 1
fi

# The archive holds a single top-level directory.
mkdir "$tmp/renode"
tar -xzf "$tmp/renode.tar.gz" -C "$tmp/renode" --strip-components=1
if [ ! -f "$tmp/renode/renode" ]; then
    echo "setup.sh: unexpected archive layout, no 'renode' launcher inside" >&2
    exit 1
fi

rm -rf .renode
mv "$tmp/renode" .renode
echo "setup.sh: Renode $VERSION installed in $(pwd)/.renode"
