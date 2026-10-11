import csv, statistics as st, sys
want=["time (us)","mspOverrideFlags","rcCommand[1]","axisRate[1]","gyroADC[1]","axisP[1]","axisI[1]","axisD[1]","axisF[1]","rcCommand[0]","axisRate[0]","gyroADC[0]","axisP[0]","axisI[0]","axisF[0]"]
D={k:[] for k in want}
with open(sys.argv[1]) as fh:
    rd=csv.reader(fh); hdr=[h.strip() for h in next(rd)]; ix=[hdr.index(k) for k in want]
    for row in rd:
        for k,i in zip(want,ix): D[k].append(float(row[i]))
N=len(D["time (us)"]); spans=[]; start=None
for i in range(N):
    e=D["mspOverrideFlags"][i]==2
    if e and start is None: start=i
    elif not e and start is not None: spans.append((start,i-1)); start=None
spans=[s for s in spans if D["time (us)"][s[1]]-D["time (us)"][s[0]]>2e6]
q=lambda v,p: sorted(v)[int(p*(len(v)-1))]
print("PITCH: span | rcCmd med | setpt med/p5/p95 (deg/s) | gyro med/p5/p95 | PID sum med/p5/p95 | |sum|>=500 % | FF share of |sum|")
for si,(a,b) in enumerate(spans,1):
    rc=D["rcCommand[1]"][a:b+1]; sp=[v for v in D["axisRate[1]"][a:b+1]]; gy=[v for v in D["gyroADC[1]"][a:b+1]]  # deg/s as logged, no /10 (flight-analysis-20261011b.md s2)
    tot=[D["axisP[1]"][i]+D["axisI[1]"][i]+D["axisD[1]"][i]+D["axisF[1]"][i] for i in range(a,b+1)]
    ff=[abs(D["axisF[1]"][i]) for i in range(a,b+1)]; at=[abs(v) for v in tot]
    print("  %d | %+5.0f | %+6.1f %+6.1f %+6.1f | %+6.1f %+6.1f %+6.1f | %+5.0f %+5.0f %+5.0f | %4.1f%% | %.2f" % (si, st.median(rc), st.median(sp), q(sp,.05), q(sp,.95), st.median(gy), q(gy,.05), q(gy,.95), st.median(tot), q(tot,.05), q(tot,.95), 100*sum(1 for v in at if v>=500)/len(at), sum(ff)/sum(at)))
print("ROLL:")
for si,(a,b) in enumerate(spans,1):
    rc=D["rcCommand[0]"][a:b+1]; sp=[v for v in D["axisRate[0]"][a:b+1]]; gy=[v for v in D["gyroADC[0]"][a:b+1]]  # deg/s as logged, no /10 (flight-analysis-20261011b.md s2)
    tot=[D["axisP[0]"][i]+D["axisI[0]"][i]+D["axisF[0]"][i] for i in range(a,b+1)]; at=[abs(v) for v in tot]
    print("  %d | %+5.0f | %+6.1f %+6.1f %+6.1f | %+6.1f %+6.1f %+6.1f | %+5.0f %+5.0f %+5.0f | %4.1f%%" % (si, st.median(rc), st.median(sp), q(sp,.05), q(sp,.95), st.median(gy), q(gy,.05), q(gy,.95), st.median(tot), q(tot,.05), q(tot,.95), 100*sum(1 for v in at if v>=500)/len(at)))
# unengaged pilot reference
a,b = 0, spans[0][0]-1
sp=[v for v in D["axisRate[1]"][a:b+1]]; gy=[v for v in D["gyroADC[1]"][a:b+1]]  # deg/s as logged, no /10 (flight-analysis-20261011b.md s2)
print("pilot pre-engage pitch setpt med %.1f p5 %.1f p95 %.1f | gyro med %.1f p5 %.1f p95 %.1f" % (st.median(sp), q(sp,.05), q(sp,.95), st.median(gy), q(gy,.05), q(gy,.95)))
