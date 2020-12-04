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

Post-Processing of logs
---------------------------

a) Save collection of *.moz_log files to a dedicated folder, e.g.,
```
    $ cd nada_eval/
    $ mkdir 2020-12-02-AUS-local-nada-owd
    $ mv ../*.moz_log 2020-12-02-AUS-local-nada-owd/
```

b) Extract relevant lines from the *.moz_log file and run python script from inside the dedicated log file folder.


For NADA-OWD mode:


```
    $ cd [gecko-dev]/nada-eval/2020-12-02-AUS-local-nada-owd/
    $ grep algo webrtc.log.child-4.moz_log | grep nada_owd > nada.tr
    $ python3 ../process_log_all.py --trfile nada.tr --scen nada_owd --scheme nada
```

or for NADA-RTT mode:

```
    $ cd [gecko-dev]/nada-eval/2020-12-02-AUS-local-nada-rtt/
    $ grep algo webrtc.log.child-4.moz_log | grep nada_rtt > nada.tr
    $ python3 ../process_log_all.py --trfile nada.tr --scen nada_rtt --scheme nada
```

or for default mode with use_transport_cc ON:

```
    $ cd [gecko-dev]/nada-eval/2020-12-02-AUS-local-default-wi-transcc/
    $ grep algo webrtc.log.child-4.moz_log | grep default > default.tr
    $ python3 ../process_log_all.py --trfile default.tr --scen default --scheme default
```

Note that the number/level of the non-empty *.moz_log file could be different, such as webrtc.log.child-2.moz_log. Use the following command to find out which one to grep from:

```
   $ ls -l *.moz_log
```


