import 'leaflet/dist/leaflet.css';
import React from 'react';
import ReactDOM from 'react-dom';
import App from './components/app';
import AppConfig from './app-config';
import { extendControlPositions } from './leaflet/control-positions';
import * as serviceWorker from './serviceWorker';

if (!process.env.REACT_APP_MOCK) {
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

  ReactDOM.render(
    <App transportFactory={AppConfig.transportFactory} />,
    document.getElementById('root'),
  );

  // If you want your app to work offline and load faster, you can change
  // unregister() to register() below. Note this comes with some pitfalls.
  // Learn more about service workers: https://bit.ly/CRA-PWA
  serviceWorker.unregister();
} else {
  import('./mock/index');
}
