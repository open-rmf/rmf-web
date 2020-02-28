export type OnClockUpdateCallback = (time: number) => Promise<void>

export class OnClockUpdate {
  onClockUpdateCallbacks: OnClockUpdateCallback[] = []

  addOnClockUpdateCallback(onClockSecond: OnClockUpdateCallback) {
    if (this.onClockUpdateCallbacks.length <= 0) {
      this.onClockUpdateCallbacks.push(onClockSecond)
    } else {
      if (this.onClockUpdateCallbacks.indexOf(onClockSecond) === -1) {
        this.onClockUpdateCallbacks.push(onClockSecond)
      }
    }
  }

  clearOnClockUpdateCallback() {
    this.onClockUpdateCallbacks = []
  }

  removeOnClockUpdateCallback(onClockSecond: OnClockUpdateCallback) {
    let index = this.onClockUpdateCallbacks.indexOf(onClockSecond)

    if (index === -1) {
      return
    }

    this.onClockUpdateCallbacks.splice(index, 1)
  }

  protected callOnClockUpdateCallbacks(time: number) {
    for (const cb of this.onClockUpdateCallbacks) {
      cb(time)
    }
  }
}

export class ClockSource extends OnClockUpdate {
  timeDiff = 0
  private intervalID = 0
  private updateInterval = 500
  private expectedTime = 0

  start() {
    if (this.intervalID) return

    this.expectedTime = Date.now()
    this.callOnClockUpdateCallbacks(this.expectedTime + this.timeDiff)
    this.intervalID = setInterval(this.update, this.updateInterval)
  }

  stop() {
    clearTimeout(this.intervalID)
  }

  private update = () => {
    this.callOnClockUpdateCallbacks(Date.now() + this.timeDiff)
  }
}