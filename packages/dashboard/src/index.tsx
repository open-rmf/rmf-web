import { LocalizationProvider } from 'react-components';
import ReactDOM from 'react-dom/client';
import { BrowserRouter } from 'react-router-dom';

import App from './components/app';

const root = ReactDOM.createRoot(document.getElementById('root') as HTMLElement);
root.render(
  <LocalizationProvider>
    <BrowserRouter>
      <App />
    </BrowserRouter>
  </LocalizationProvider>,
);
