project('sdr_fm_rx', ['cpp'],
  version : '1.0',
  meson_version : '>=1.1.0',
  default_options : ['warning_level=2', 'cpp_std=c++11'])

src = [
  'fm_rx.cpp'
]

cpp = meson.get_compiler('cpp')
# libalsa = dependency('alsa')
libalsa = cpp.find_library('asound', static : false)

rtlsdr = get_option('rtlsdr')
if rtlsdr
    add_project_arguments('-DRTLSDR', language: 'cpp')
    libsdr = dependency('librtlsdr')
else
    libsdr = dependency('libiio')
endif

cpp_args = [
  '-Wall',
  '-O3',
]

executable('fm_rx',
           src,
           install : true,
           dependencies: [libsdr, libalsa],
           cpp_args : cpp_args)

