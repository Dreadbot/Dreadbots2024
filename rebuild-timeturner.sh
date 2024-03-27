#!/bin/bash

TIMETURNER_ROOT=${1:-$HOME}

if [ "$USER" != "vision" ]; then
    printf "\x1b[1;31mMust be run as the 'vision' user.\x1b[0m\n"
    exit 1
fi

if ip route | grep -qE "default.*?dev eth0"; then
    sudo ip route del default dev eth0 # disable automatic routing over ethernet, allowing internet access
fi

printf "\x1b[1;34mChecking for rust toolchain... \x1b[0m"
if ! command -v rustup >/dev/null 2&>1; then
    printf "\x1b[1;31mnot found\x1b[0;m\n"
    printf "\x1b[1;34mInstalling rustup\x1b[0m\n"
    curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | sh
else
    printf "\x1b[1;32mfound\x1b[0;m\n"
fi

printf "\x1b[1;34mChecking for maturin... \x1b[0m"
if ! command -v maturin >/dev/null 2&>1; then
    printf "\x1b[1;34mInstalling maturin\x1b[0m\n"
    pip install maturin --break-system-packages || exit
else
    printf "\x1b[1;32mfound\x1b[0;m\n"
fi

pushd "$TIMETURNER_ROOT/timeturner" >/dev/null || exit
printf "\x1b[1;34mBuilding timeturner lib\x1b[0m\n"
cargo build --release || exit
printf "\x1b[1;34mBuilding python bindings\x1b[0m\n"
maturin build --release || exit
for WHL in target/wheels/*; do
    printf "\x1b[1;34mInstalling wheel %s\x1b[0m\n" "$WHL"
    pip install "$WHL" --break-system-packages --force-reinstall || exit
done
popd >/dev/null || exit

pushd "$TIMETURNER_ROOT/timeturner-server" >/dev/null || exit
printf "\x1b[1;34mBuilding timeturner server\x1b[0m\n"
cargo build --release || exit
printf "\x1b[1;34mHardlinking server bin to /usr/bin/timeturner-server\n"
sudo rm /usr/bin/timeturner-server
sudo ln target/release/timeturner-server /usr/bin/timeturner-server || exit
popd >/dev/null || exit

printf "\x1b[1;34mDon't forget to restart the server service\x1b[0m\n"
