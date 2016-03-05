package = "torch-ros"
version = "scm-1"

source = {
   url = "git://github.com/Xamla/torch-ros.git",
}

description = {
   summary = "ROS bindings for Torch",
   detailed = [[
   ]],
   homepage = "https://github.com/Xamla/torch-ros",
   license = "BSD"
}

dependencies = {
   "torch >= 7.0",
   "md5 >= 1.2-1"
}

build = {
   type = "command",
   build_command = [[
cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE)
]],
   install_command = "cd build && $(MAKE) install"
}
