export type AsyncEventListener<T = Event> = (evt: T) => Promise<void>

export class WebSocketManager {
  private onOpenCallbacks: AsyncEventListener[] = []
  private onMessageCallbacks: AsyncEventListener<WebSocketMessageEvent>[] = []

  private __client?: WebSocket

  constructor(private url: string) {}

  get client() {
    return this.__client
  }

  connect() {
    this.__client = new WebSocket(this.url)
    this.__client.onopen = (event) => {
      for (const cb of this.onOpenCallbacks) {
        cb(event)
      }
    }
    this.__client.onmessage = (event) => {
      for (const cb of this.onMessageCallbacks) {
        cb(event)
      }
    }
    return this.__client
  }

  disconnect() {
    if (!this.__client) return

    return this.__client.close()
  }

  addOnOpenCallback(eventListener: AsyncEventListener) {
    const callbackIndex = this.onOpenCallbacks.indexOf(eventListener)

    if (callbackIndex !== -1) return

    this.onOpenCallbacks.push(eventListener)
  }

  removeOnOpenCallback(eventListener: AsyncEventListener) {
    const callbackIndex = this.onOpenCallbacks.indexOf(eventListener)

    if (callbackIndex === -1) return

    this.onOpenCallbacks.splice(callbackIndex, 1)
  }

  clearOnOpenCallbacks() {
    this.onOpenCallbacks = []
  }

  addOnMessageCallback(eventListener: AsyncEventListener<WebSocketMessageEvent>) {
    const callbackIndex = this.onMessageCallbacks.indexOf(eventListener)

    if (callbackIndex !== -1) return

    this.onMessageCallbacks.push(eventListener)
  }

  removeOnMessageCallback(eventListener: AsyncEventListener<WebSocketMessageEvent>) {
    const callbackIndex = this.onMessageCallbacks.indexOf(eventListener)

    if (callbackIndex === -1) return

    this.onMessageCallbacks.splice(callbackIndex, 1)
  }

  clearOnMessageCallbacks() {
    this.onMessageCallbacks = []
  }
}