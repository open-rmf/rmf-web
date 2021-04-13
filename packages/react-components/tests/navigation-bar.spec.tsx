import React from 'react';
import NavigationBar from '../lib/navigation-bar';
import { render, screen } from '@testing-library/react';
import { BrowserRouter } from 'react-router-dom';

describe('Navigation Bar', () => {
  it('renders the title bar', () => {
    const root = render(
      <BrowserRouter>
        <NavigationBar />
      </BrowserRouter>,
    );
    expect(screen.getByLabelText('navigation-tabs')).toBeTruthy();
    root.unmount();
  });
});
