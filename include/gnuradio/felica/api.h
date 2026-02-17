#ifndef INCLUDED_GNURADIO_FELICA_API_H
#define INCLUDED_GNURADIO_FELICA_API_H

#ifdef _MSC_VER
#ifdef felica_EXPORTS
#define FELICA_API __declspec(dllexport)
#else
#define FELICA_API __declspec(dllimport)
#endif
#else
#define FELICA_API __attribute__((visibility("default")))
#endif

#endif
