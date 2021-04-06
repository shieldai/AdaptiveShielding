import csv
import sys

import numpy as np
import matplotlib.pyplot as plt

TSMAX=10000

class LogData:

    def __init__(self, logfile, label):
        self.label = label
        self.ts = []
        self.vehicleNumber = []
        self.departedNumber = []
        self.arrivedNumber = []
        self.totalHaltingNumber = []
        self.totalVehicleNumber = []
        self.totalMeanSpeed = []
        self.costs = []
        self.waitTimePerVehicle = []
        self.accumulatedWaitTimePerVehicle = []
        self.waitTime = []
        self.accumulatedWaitTime = []

        self.busVehicleNumber = []
        self.busWaitTime = []
        self.busAccumulatedWaitTime = []

        count = 0

        print("Read", logfile)
        with open(logfile, newline='\n') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',')
            for row in spamreader:
                assert row != 11

                if count >= TSMAX:
                    break

                self.ts.append(int(row[0]))
                self.vehicleNumber.append(int(row[1]))
                self.departedNumber.append(int(row[2]))
                self.arrivedNumber.append(int(row[3]))
                self.totalHaltingNumber.append(int(row[4]))
                self.totalVehicleNumber.append(int(row[5]))
                self.totalMeanSpeed.append(float(row[6]))
                self.costs.append(float(row[7]))
                self.waitTimePerVehicle.append(float(row[8]))
                self.accumulatedWaitTimePerVehicle.append(float(row[9]))
                self.waitTime.append(float(row[10]))
                self.accumulatedWaitTime.append(float(row[11]))

                self.busVehicleNumber.append(float(row[12]))
                self.busWaitTime.append(float(row[13]))
                self.busAccumulatedWaitTime.append(float(row[14]))
                count += 1

    def readShieldLog(self, shieldLog):

        self.probabilities = []
        self.probDelta = []
        self.stateSpace = []
        self.stateDelta = []
        self.generation = []
        self.deviation = []
        self.update = []

        count = 0

        print("Read shield log", shieldLog)
        with open(shieldLog, newline='\n') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',')
            for row in spamreader:
                assert row != 10

                if row[0][0] == '#':
                    continue

                if count >= TS:
                    break

                time = row[0]
                lastStepHaltingNumbers = row[1][1:-1].split(' ')
                currentJunctionPhase = row[2]
                StateSpace = row[3][1:-1].split(' ')
                stateDelta = row[4]
                Probabilities = row[5][1:-1].split(' ')
                probDelta = row[6]
                generation = row[7]
                shieldUpdated = row[8]
                dev = row[9]

                self.probabilities.append(Probabilities)
                self.probDelta.append(probDelta)
                self.stateSpace.append(StateSpace)
                self.stateDelta.append(stateDelta)
                self.generation.append(generation)
                self.deviation.append(dev)
                self.update.append(shieldUpdated)
                count += 1

data = []

if len(sys.argv) <= 1:
    print("Please provide some log files")
    exit(-1)

logFiles = sys.argv[1:]

for lf in logFiles:
    data.append(LogData(lf, lf.split('.')[0]))


fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)

for d in data:
    print("plot ", d.label)
    ax1.plot(d.ts, d.waitTime, label=d.label)
    ax2.plot(d.ts, d.accumulatedWaitTime, label=d.label)
    ax3.plot(d.ts, d.busWaitTime, label=d.label)
    ax4.plot(d.ts, d.busAccumulatedWaitTime, label=d.label)

for i in range(0, len(d.ts), 500):
    ax1.vlines(i,0,max(d.waitTime))
    ax2.vlines(i,0,max(d.accumulatedWaitTime))
    ax3.vlines(i,0,max(d.busWaitTime))
    ax4.vlines(i,0,max(d.busAccumulatedWaitTime))


ax1.set_title('waitTime')
ax1.set(label='y-label')
ax1.set_yscale('log')

ax2.set_title('accumulatedWaitTime')
ax2.set(label='y-label')
ax2.set_yscale('log')

ax3.set_title('waitTime BUS')
ax3.set(label='y-label')
ax3.set_yscale('log')

ax4.set_title('accumulatedWaitTime BUS')
ax4.set(label='y-label')
ax4.set_yscale('log')

fig.tight_layout()

plt.xlabel('timestep')
plt.legend()
plt.show()