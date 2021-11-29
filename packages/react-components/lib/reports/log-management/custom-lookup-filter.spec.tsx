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

  it('triggers a new search closing the filter selector', () => {
    render(
      <CustomLookupFilter
        selectedFilter={['error']}
        lookup={{ error: 'error', warn: 'warn' }}
        onFilterChange={mockOnChange}
      />,
    );

    // click on existing selected filter "error" to open up input to expose "warn"
    userEvent.click(screen.getByText('error'));
    userEvent.click(screen.getByText('warn'));
    userEvent.type(screen.getByText('warn'), '{esc}');
    expect(mockOnChange).toHaveBeenCalled();
  });
});
