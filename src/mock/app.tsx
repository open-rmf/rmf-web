import React from 'react';
import ReactDOM from 'react-dom';
import AppConfig from '../app-config';
import App from '../components/app';
import { extendControlPositions } from '../leaflet/control-positions';

extendControlPositions();

ReactDOM.render(
  <App transportFactory={AppConfig.transportFactory} />,
  document.getElementById('root'),
);
