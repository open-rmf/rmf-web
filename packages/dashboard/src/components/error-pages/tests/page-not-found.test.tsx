import React from 'react';
import { render } from '@testing-library/react';
import NotFoundPage from '../page-not-found';
import { BrowserRouter } from 'react-router-dom';

describe('PageNotFound', () => {
  test('renders correctly', () => {
    const root = render(
      <BrowserRouter>
        <NotFoundPage />
      </BrowserRouter>,
    );
    root.unmount();
  });
});
