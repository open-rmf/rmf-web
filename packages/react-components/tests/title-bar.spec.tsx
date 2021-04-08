import React from 'react';
import TitleBar from '../lib/title-bar';
import { render, screen } from '@testing-library/react';

describe('Title Bar', () => {
  it('renders the title bar', () => {
    render(<TitleBar logoPath="../resources/roshealth-logo-white.png" />);
    expect(screen.getByAltText('logo')).toBeTruthy();
    expect(screen.getByText('Powered by OpenRMF')).toBeTruthy();
  });
});
