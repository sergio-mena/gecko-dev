'''
    process_log_owd.py


    python3 ../process_log_all.py --scen nada_owd --trfile nada.tr --scheme nada
'''

import re
import sys
import numpy as np
import matplotlib.pyplot as plt
import argparse

class FBStats:
    def __init__(self, scheme='default'):
        self.ts = 0.
        self.fbint = 0.
        self.qdel = 0.
        self.rtt = 0.
        self.nloss = 0
        self.plr = 0.
        self.rmode = 0
        self.xcurr = 0.
        self.srate = 0.
        self.rrate = 0.
        self.rmin = 0.
        self.rmax = 0.
        self.scheme = scheme

    def process_line_default(self, line):
        '''
        sample line from default.tr:

        2020-11-13 04:30:19.892468 UTC - [Child 92835: Socket Thread]: D/webrtc_trace (delay_based_bwe.cc:383):
            DlayBasedBwe::MaybeUpdateEstimate | algo:default  | ts: 253 ms | fbint: 51 ms |
            qdel: 2 ms | rtt: 32 ms | ploss: 0 | plr: 0.000000 % |
            update: 1 | probe: 0 | rrate: 770.352941 Kbps | srate: 5324.241000 Kbps
        '''

        'parsing per-feedback interval stats'
        match_ts    = re.search(r' ts: (\d+) ms', line);                         print('match_ts: ', match_ts);          # ts: 1421 ms
        match_fbint = re.search(r' fbint: (\d+) ms', line);                      print('match_fbint: ', match_fbint);    # fbint: 500 ms
        match_qdel  = re.search(r' qdel: (\d+) ms', line);                       print('match_qdel: ', match_qdel);      # qdel: 0 ms
        match_rtt   = re.search(r' rtt: (\d+) ms', line);                        print('match_rtt: ', match_rtt);        # rtt: 7 ms
        match_nloss = re.search(r' nloss: (\d+)', line);                         print('match_nloss: ', match_nloss);    # nloss: 0
        match_plr   = re.search(r' plr: (\d+(?:\.\d*)?|\.\d+) %', line);         print('match_plr: ', match_plr);        # plr: 0.000000 %
#        match_rmode = re.search(r' rmode: (\d+)', line);                         print('match_rmode: ', match_rmode);    # rmode: 0
#        match_xcurr = re.search(r' xcurr: (\d+(?:\.\d*)?|\.\d+) ms', line);      print('match_xcurr: ', match_xcurr);    # xcurr: 7.000000 ms
        match_rrate = re.search(r' rrate: (\d+(?:\.\d*)?|\.\d+) Kbps', line);    print('match_rrate: ', match_rrate);    # rrate: 1196.224000 Kbps
        match_srate = re.search(r' srate: (\d+(?:\.\d*)?|\.\d+) Kbps', line);    print('match_srate: ', match_srate);    # srate: 720.000000 Kbps
#        match_rmin  = re.search(r' rmin: (\d+(?:\.\d*)?|\.\d+) Kbps', line);     print('match_rmin: ', match_rmin);      # rmin: 10.000000 Kbps
#        match_rmax  = re.search(r' rmax: (\d+(?:\.\d*)?|\.\d+) Kbps', line);     print('match_rmax: ', match_rmax);      # rmax: 3000.000000 Kbps

        logvec = []
        if match_ts:
            self.ts = float(match_ts.group(1))/1000.  # ms -> sec
            self.fbint = float(match_fbint.group(1))
            self.qdel = float(match_qdel.group(1))
            self.rtt = float(match_rtt.group(1))
            self.nloss = float(match_nloss.group(1))
            self.plr = float(match_plr.group(1))
            # self.rmode = int(match_rmode.group(1))
            # self.xcurr = float(match_xcurr.group(1))
            self.rrate = float(match_rrate.group(1))
            self.srate = float(match_srate.group(1))
            # self.rmin = float(match_rmin.group(1))
            # self.rmax = float(match_rmax.group(1))

            logvec.append(self.ts)          # 0: ts
            logvec.append(self.fbint)       # 1: fbint
            logvec.append(self.qdel)        # 2: qdel
            logvec.append(self.rtt)         # 3: rtt
            logvec.append(self.nloss)       # 4: nloss
            logvec.append(self.plr)         # 5: plr
            logvec.append(self.rmode)       # 6: rmode, use init value as filler
            logvec.append(self.xcurr)       # 7: xcurr, use init value as filler
            logvec.append(self.rrate)       # 8: receiving rate
            logvec.append(self.srate)       # 9: sending rate
        else:
            print('mismatch ...:', line)

        return logvec


    def process_line_nada(self, line):

        '''
        sample line from NADA-RTT:


        2020-11-13 04:26:14.738311 UTC - [Child 92229: Socket Thread]: D/webrtc_trace (nada_core.cc:371):
            NADA Update | algo: nada_rtt | ts: 1421 ms | fbint: 500 ms | qdel: 0 ms | dfwd: 0 ms |
            relrtt: 7 ms | rtt: 7 ms | nloss: 0 | plr: 0.000000 % | rmode: 0 | xcurr: 7.000000 ms |
            rrate: 1196.224000 Kbps | srate: 720.000000 Kbps | rmin: 10.000000 Kbps | rmax: 3000.000000 Kbps

        sample line from NADA-OWD:

        2020-11-13 04:21:00.595332 UTC - [Child 91364: Socket Thread]: D/webrtc_trace (nada_core.cc:371):
        NADA Update | algo: nada_owd | ts: 219 ms | fbint: 120 ms | qdel: 0 ms | dfwd: 83 ms |
        relrtt: 13 ms | rtt: 122 ms | nloss: 0 | plr: 0.000000 % | rmode: 0 | xcurr: 0.000000 ms |
        rrate: 354.976750 Kbps | srate: 1177.166000 Kbps | rmin: 300.000000 Kbps | rmax: 3000.000000 Kbps

        '''

        'parsing per-feedback interval stats'
        match_ts    = re.search(r' ts: (\d+) ms', line);                         print('match_ts: ', match_ts);          # ts: 1421 ms
        match_fbint = re.search(r' fbint: (\d+) ms', line);                      print('match_fbint: ', match_fbint);    # fbint: 500 ms
        match_qdel  = re.search(r' qdel: (\d+) ms', line);                       print('match_qdel: ', match_qdel);      # qdel: 0 ms
        match_rtt   = re.search(r' rtt: (\d+) ms', line);                        print('match_rtt: ', match_rtt);        # rtt: 7 ms
        match_nloss = re.search(r' nloss: (\d+)', line);                         print('match_nloss: ', match_nloss);    # nloss: 0
        match_plr   = re.search(r' plr: (\d+(?:\.\d*)?|\.\d+) %', line);         print('match_plr: ', match_plr);        # plr: 0.000000 %
        match_rmode = re.search(r' rmode: (\d+)', line);                         print('match_rmode: ', match_rmode);    # rmode: 0
        match_xcurr = re.search(r' xcurr: (\d+(?:\.\d*)?|\.\d+) ms', line);      print('match_xcurr: ', match_xcurr);    # xcurr: 7.000000 ms
        match_rrate = re.search(r' rrate: (\d+(?:\.\d*)?|\.\d+) Kbps', line);    print('match_rrate: ', match_rrate);    # rrate: 1196.224000 Kbps
        match_srate = re.search(r' srate: (\d+(?:\.\d*)?|\.\d+) Kbps', line);    print('match_srate: ', match_srate);    # srate: 720.000000 Kbps
        match_rmin  = re.search(r' rmin: (\d+(?:\.\d*)?|\.\d+) Kbps', line);     print('match_rmin: ', match_rmin);      # rmin: 10.000000 Kbps
        match_rmax  = re.search(r' rmax: (\d+(?:\.\d*)?|\.\d+) Kbps', line);     print('match_rmax: ', match_rmax);      # rmax: 3000.000000 Kbps

        logvec = []
        if match_ts:
            self.ts = float(match_ts.group(1))/1000.  # ms -> sec
            self.fbint = float(match_fbint.group(1))
            self.qdel = float(match_qdel.group(1))
            self.rtt = float(match_rtt.group(1))
            self.nloss = float(match_nloss.group(1))
            self.plr = float(match_plr.group(1))
            self.rmode = int(match_rmode.group(1))
            self.xcurr = float(match_xcurr.group(1))
            self.rrate = float(match_rrate.group(1))
            self.srate = float(match_srate.group(1))
            self.rmin = float(match_rmin.group(1))
            self.rmax = float(match_rmax.group(1))

            logvec.append(self.ts)          # 0: ts
            logvec.append(self.fbint)       # 1: fbint
            logvec.append(self.qdel)        # 2: qdel
            logvec.append(self.rtt)         # 3: rtt
            logvec.append(self.nloss)       # 4: nloss
            logvec.append(self.plr)         # 5: plr
            logvec.append(self.rmode)       # 6: rmode
            logvec.append(self.xcurr)       # 7: xcurr
            logvec.append(self.rrate)       # 8: receiving rate
            logvec.append(self.srate)       # 9: sending rate
        else:
            print('mismatch ...:', line)

        return logvec

# def process_line(line, s):

#     '''
#     sample line:

#     2020-11-13 04:26:14.738311 UTC - [Child 92229: Socket Thread]: D/webrtc_trace (nada_core.cc:371):
#          NADA Update | algo: nada_rtt | ts: 1421 ms | fbint: 500 ms | qdel: 0 ms | dfwd: 0 ms | relrtt: 7 ms |
#          rtt: 7 ms | nloss: 0 | plr: 0.000000 % | rmode: 0 | xcurr: 7.000000 ms | rrate: 1196.224000 Kbps |
#          srate: 720.000000 Kbps | rmin: 10.000000 Kbps | rmax: 3000.000000 Kbps

#     '''

#     'parsing per-feedback interval stats'
#     match_ts    = re.search(r' ts: (\d+) ms', line);                         print('match_ts: ', match_ts);          # ts: 1421 ms
#     match_fbint = re.search(r' fbint: (\d+) ms', line);                      print('match_fbint: ', match_fbint);    # fbint: 500 ms
#     match_qdel  = re.search(r' qdel: (\d+) ms', line);                       print('match_qdel: ', match_qdel);      # qdel: 0 ms
#     match_rtt   = re.search(r' rtt: (\d+) ms', line);                        print('match_rtt: ', match_rtt);        # rtt: 7 ms
#     match_nloss = re.search(r' nloss: (\d+)', line);                         print('match_nloss: ', match_nloss);    # nloss: 0
#     match_plr   = re.search(r' plr: (\d+(?:\.\d*)?|\.\d+) %', line);         print('match_plr: ', match_plr);        # plr: 0.000000 %
#     match_rmode = re.search(r' rmode: (\d+)', line);                         print('match_rmode: ', match_rmode);    # rmode: 0
#     match_xcurr = re.search(r' xcurr: (\d+(?:\.\d*)?|\.\d+) ms', line);      print('match_xcurr: ', match_xcurr);    # xcurr: 7.000000 ms
#     match_rrate = re.search(r' rrate: (\d+(?:\.\d*)?|\.\d+) Kbps', line);    print('match_rrate: ', match_rrate);    # rrate: 1196.224000 Kbps
#     match_srate = re.search(r' srate: (\d+(?:\.\d*)?|\.\d+) Kbps', line);    print('match_srate: ', match_srate);    # srate: 720.000000 Kbps
#     match_rmin  = re.search(r' rmin: (\d+(?:\.\d*)?|\.\d+) Kbps', line);     print('match_rmin: ', match_rmin);      # rmin: 10.000000 Kbps
#     match_rmax  = re.search(r' rmax: (\d+(?:\.\d*)?|\.\d+) Kbps', line);     print('match_rmax: ', match_rmax);      # rmax: 3000.000000 Kbps

#     if match_ts:
#         s.ts = float(match_ts.group(1))/1000.  # ms -> sec
#         s.fbint = float(match_fbint.group(1))
#         s.qdel = float(match_qdel.group(1))
#         s.rtt = float(match_rtt.group(1))
#         s.nloss = float(match_nloss.group(1))
#         s.plr = float(match_plr.group(1))
#         s.rmode = int(match_rmode.group(1))
#         s.xcurr = float(match_xcurr.group(1))
#         s.rrate = float(match_rrate.group(1))
#         s.srate = float(match_srate.group(1))
#         s.rmin = float(match_rmin.group(1))
#         s.rmax = float(match_rmax.group(1))
#     else:
#         print('mismatch ...:', line)

#     return s

def plot_log(logmat, pngfile):

    '''

    logmat:
    0: ts
    1: fbint
    2: qdel
    3: rtt
    4: nloss
    5: plr
    6: rmode
    7: xcurr
    8: rrate
    9: srate
    '''


    print('calculate receiver-reported sending rate')
    # # pktmat[:,1] = pktmat[:,1]-pktmat[0,1]
    # tmax = pktmat[-1,1]
    # tdiff = 200.
    # tslist = np.arange(0, tmax, tdiff)
    # psizelist = []
    # for t in tslist:
    #     # print('calculating average rate for time %.1f' % t)
    #     psizetmp = pktmat[np.where(pktmat[:,1]<=t), 3];
    #     psizelist.append(np.sum(psizetmp))

    # psizelist = np.asarray(psizelist)
    # psizelist = np.diff(psizelist)
    # rrate = psizelist*8/tdiff;   # in Kbps
    # tslist = tslist[:-1]/1000;   # ms -> s

    # np.savetxt('rrate.txt', rrate, fmt='%6.2f ')
    # np.savetxt('tslist.txt', tslist, fmt='%6.2f ')

    'plot data'
    plt.figure()
    plt.subplot(311)
    plt.plot(logmat[:,0], logmat[:,9], 'b-o', mec='blue', ms=4, label='target')
    plt.plot(logmat[:,0], logmat[:,8], 'rd', mec='red', ms=2, label='receiver reported')
    # plt.plot(tslist, rrate,  'rd', mec='red', ms=2, label='receiver reported')
    plt.legend(loc='upper left', ncol=2)
    plt.ylim(0, 6000)
    plt.ylabel('Rate (Kbps)')
    plt.subplot(312)
    plt.plot(logmat[:,0], logmat[:,2], 'b-o', mec='blue', ms=4, label='queuing delay')
    plt.plot(logmat[:,0], logmat[:,3], 'ms',  mec='none', ms=2, label='rtt')
    # plt.plot(logmat[:,0], logmat[:,7], 'rd', mec='red', ms=2, label='xcurr')
    plt.legend(loc='upper left', ncol=2)
    plt.ylim(0, )
    plt.ylabel('Delay/RTT (ms)')
    plt.subplot(313)
    plt.plot(logmat[:,0], logmat[:,5], 'b-o', mec='blue', ms=4, label='loss ratio')
    plt.legend()
    plt.ylim(0, )
    plt.ylabel('PLR (%)')
    plt.xlabel('Time (s)')
    plt.savefig(pngfile)
    plt.close()


def main(args):

    logfile = '%s.txt' % args.scen
    pngfile = '%s.png' % args.scen
    #  logfile2 = '%s.txt' % args.pktfile[:-3]

    s = FBStats()
    logmat = []
    with open(args.trfile, 'r') as f:
        for line in f:
            if args.verbose:
                print(line)

            if args.scheme == 'nada':
                logvec = s.process_line_nada(line)
            elif args.scheme == 'default':
                logvec = s.process_line_default(line)
            else:
                print('unsupported scheme {}'.format(args.scheme))
            # logvec = [s.ts, s.fbint, s.qdel, s.rtt, s.nloss, s.plr, s.rmode, s.xcurr, s.rrate, s.srate]
            logmat.append(logvec)

    logmat = np.asarray(logmat)

    'save to plain txt log'
    linefmt = '%8.2f %6d %6d %6d %6d %6.2f %4d %8.2f %8.2f %8.2f'
    np.savetxt(logfile, logmat, fmt=linefmt)


    'plot log traces'
    plot_log(logmat, pngfile)
######################## MAIN ######################

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--trfile', help='trace filename for rate update info and stats', default='nada_owd.tr')
#   parser.add_argument('--pktfile', help='trace filename for per-pkt info', default='pktinfo.tr')
    parser.add_argument('--scheme', help='scheme name: default | nada', default='default')
    parser.add_argument('--scen',   help='scenario name for plotting', default='scen')
    parser.add_argument('--verbose', default=False, action='store_true')
    args = parser.parse_args()

    main(args)


