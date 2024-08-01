import { fireEvent, render } from '@testing-library/react';

import { MutexGroupData, MutexGroupTable } from './mutex-group-table';

describe('Mutex group table', () => {
  it('shows all mutex groups and responds to clicks', () => {
    const mutexGroups: MutexGroupData[] = [
      {
        name: 'group1',
        lockedBy: 'fleet1/robot1',
        requestedBy: ['fleet1/robot2', 'fleet2/robot1'],
      },
      {
        name: 'group2',
        lockedBy: 'fleet1/robot1',
        requestedBy: ['fleet1/robot2'],
      },
    ];
    const group1Click = vi.fn();
    const group2Click = vi.fn();
    const root = render(
      <MutexGroupTable
        mutexGroups={mutexGroups}
        onMutexGroupClick={(_ev, mutexGroup) => {
          if (mutexGroup.name === 'group1') {
            group1Click();
          } else if (mutexGroup.name === 'group2') {
            group2Click();
          }
        }}
      />,
    );

    expect(root.getByText('group1')).toBeTruthy();
    expect(root.getByText('group2')).toBeTruthy();

    fireEvent.click(root.getByText('group1'));
    expect(group1Click).toHaveBeenCalled();
    fireEvent.click(root.getByText('group2'));
    expect(group2Click).toHaveBeenCalled();
  });
});
