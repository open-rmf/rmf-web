import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { SossTransport } from '@osrf/romi-js-soss-transport';
import 'leaflet/dist/leaflet.css';
import React from 'react';
import ReactDOM from 'react-dom';
import App from './App';
import AppConfig from './app-config';
import './index.css';
import { extendControlPositions } from './leaflet/control-positions';
import './mock/doors-panel';
import { FakeAuthService } from './mock/fake-auth-service';
import * as serviceWorker from './serviceWorker';

// import { WebSocketManager } from './util/websocket';

extendControlPositions();
// export const webSocketManager = new WebSocketManager('ws://localhost:8006')

/*
webSocketManager.addOnOpenCallback(async (event: Event) => {
  if (!webSocketManager.client) return
  webSocketManager.client.send(JSON.stringify({
    request: 'time',
    param: {}
  }))
})
*/

// webSocketManager.connect()

/*
webSocketManager.addOnMessageCallback(async (event: WebSocketMessageEvent) => {
  clockSource.timeDiff =
})
*/

const auth = new FakeAuthService(AppConfig.authUrl);

function transportFactory(): Promise<RomiCore.Transport> {
  return SossTransport.connect('romi-dashboard', AppConfig.authUrl, auth.token());
}

ReactDOM.render(<App transportFactory={transportFactory} />, document.getElementById('root'));

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
