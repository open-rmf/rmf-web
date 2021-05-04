import React from 'react';
import { render, cleanup, screen } from '@testing-library/react';
import { BannerTab } from '../lib/banner-tab';

describe('Banner Tab', () => {
  it('renders correctly', () => {
    render(<BannerTab logoPath={'../stories/resources/roshealth-logo-white.png'} />);
    expect(screen.getByAltText('logo')).toBeTruthy();
    cleanup();
  });
});
