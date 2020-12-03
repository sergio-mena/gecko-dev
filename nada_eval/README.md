Evaluation the NADA Congestion Control in Mozilla
===================================================

Run-Time Configurations in Nightly
----------------------------

a) Enable application-layer logging in Nightly: 

```
    $ export MOZ_LOG=webrtc_trace:4,timestamp
    $ export MOZ_LOG_FILE=webrtc.log
```

b) Run Nightly from command line: 

```
    $ ./mach run
```

c) Configure use_nada and use_transport_cc flags in browser: 

Open a tab with `about:config` as the address line. Search for `use_nada` and `use_transport_cc` in the search bar and toggle the flags accordingly. 


