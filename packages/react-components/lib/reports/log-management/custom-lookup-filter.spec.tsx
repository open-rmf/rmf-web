import { cleanup, render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { CustomLookupFilter } from './custom-lookup-filter';

describe('Custom Lookup filter', () => {
  let mockOnChange: ReturnType<typeof jasmine.createSpy>;
  beforeEach(() => {
    mockOnChange = jasmine.createSpy();
  });

  afterEach(() => {
    cleanup();
  });

  it('does not trigger a new search on filter change', () => {
    render(
      <CustomLookupFilter
        selectedFilter={['error']}
        lookup={{ error: 'error', warn: 'warn' }}
        setSelectedFilter={mockOnChange}
      />,
    );
    userEvent.click(screen.getByText('error'));
    userEvent.click(screen.getByText('warn'));
    expect(mockOnChange).toHaveBeenCalledTimes(0);
  });

  it('triggers a new search closing the filter selector', () => {
    render(
      <CustomLookupFilter
        selectedFilter={['error']}
        lookup={{ error: 'error', warn: 'warn' }}
        setSelectedFilter={mockOnChange}
      />,
    );
    userEvent.click(screen.getByText('error'));
    userEvent.click(screen.getByText('warn'));
    userEvent.type(screen.getByText('warn'), '{esc}');
    expect(mockOnChange).toHaveBeenCalledTimes(1);
  });
});
