import 'leaflet/dist/leaflet.css';
import ReactDOM from 'react-dom/client';
import { LocalizationProvider } from 'react-components';
import { BrowserRouter } from 'react-router-dom';
import App from './components/app';
import * as serviceWorker from './serviceWorker';

const root = ReactDOM.createRoot(document.getElementById('root') as HTMLElement);
root.render(
  <LocalizationProvider>
    <BrowserRouter>
      <App />
    </BrowserRouter>
  </LocalizationProvider>,
);

// NOTE: There is no point to have the app work offline as all the data must come from rmf and
// cannot be cached.

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
