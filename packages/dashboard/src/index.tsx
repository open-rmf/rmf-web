import 'leaflet/dist/leaflet.css';
import React from 'react';
import ReactDOM from 'react-dom';
import App from './components/app';
import { extendControlPositions } from './leaflet/control-positions';
import * as serviceWorker from './serviceWorker';

extendControlPositions();

ReactDOM.render(<App />, document.getElementById('root'));

// NOTE: There is no point to have the app work offline as all the data must come from rmf and
// cannot be cached.

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
