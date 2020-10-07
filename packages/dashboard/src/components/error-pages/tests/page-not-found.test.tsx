import { mount } from 'enzyme';
import React from 'react';
import NotFoundPage from '../page-not-found';
import { BrowserRouter } from 'react-router-dom';

describe('PageNotFound', () => {
  test('renders correctly', () => {
    const root = mount(
      <BrowserRouter>
        <NotFoundPage />
      </BrowserRouter>,
    );
    expect(root.html()).toMatchSnapshot();
  });
});
