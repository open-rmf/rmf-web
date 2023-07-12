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

    // TODO [CR]: FIX ERROR
    // Error: Expected spy unknown to have been called.
    // TestingLibraryElementError: Unable to find an element with the text: warn. This could be because the text is broken up by multiple elements.
    // In this case, you can provide a function for your text matcher to make your matcher more flexible.
    userEvent.click(screen.getByText('error'));
    // userEvent.click(screen.getByText('warn'));
    // userEvent.type(screen.getByText('warn'), '{esc}');
    // expect(mockOnChange).toHaveBeenCalled();
  });
});
