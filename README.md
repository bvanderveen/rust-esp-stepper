# Rust ESP32 project template

This is a project template for the use of [ctron/rust-esp-container](https://github.com/ctron/rust-esp-container).


docker run -ti -v $PWD:/home/project quay.io/ctron/rust-esp bash
create-project
make menuconfig
then you're free to run...
build-project && flash-project
bindgen-project
