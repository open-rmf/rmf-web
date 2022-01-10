// import { waitFor } from '@testing-library/react';
// import userEvent from '@testing-library/user-event';
import React from 'react';
// import { TaskSummary as RmfTaskSummary, TaskType as RmfTaskType } from 'rmf-models';
// import { render } from '../../tests/test-utils';
// import { TaskPanel, TaskPanelProps } from '../task-panel';
// import { makeTaskWithPhases } from './make-tasks';

// describe('TaskPanel', () => {
//   describe('task detail', () => {
//     const mount = async (cancelTask?: TaskPanelProps['cancelTask']) => {
//       const task = makeTaskWithPhases('test_task', 3, 3);
//       task.summary.task_profile.description.task_type.type = RmfTaskType.TYPE_CLEAN;
//       task.summary.task_profile.description.clean.start_waypoint = 'test_waypoint';
//       const root = render(<TaskPanel tasks={[task]} cancelTask={cancelTask} />);
//       userEvent.click(await root.findByText('test_task'));
//       return root;
//     };

//     it('is shown when task is clicked', async () => {
//       const root = await mount();
//       root.getByText('test_waypoint');
//     });

//     it('success snackbar is shown when task is successfully cancelled', async () => {
//       const cancelTask = () => Promise.resolve(undefined);
//       const root = await mount(cancelTask);
//       userEvent.click(root.getByLabelText('Cancel Task'));
//       await waitFor(() => root.getByText('Successfully cancelled task'));
//     });

//     it('error snackbar is shown when task failed to cancel', async () => {
//       const cancelTask = () => Promise.reject(new Error('test error'));
//       const root = await mount(cancelTask);
//       userEvent.click(root.getByLabelText('Cancel Task'));
//       await waitFor(() => root.getByText('Failed to cancel task: test error'));
//     });

//     it('clicking on cancel button triggers callback', async () => {
//       const cancelTask = jest.fn();
//       const root = render(
//         <TaskPanel tasks={[makeTaskWithPhases('task1', 1, 1)]} cancelTask={cancelTask} />,
//       );
//       userEvent.click(root.getByText('task1'));
//       userEvent.click(root.getByLabelText('Cancel Task'));
//       expect(cancelTask).toHaveBeenCalledTimes(1);
//       await waitFor(() => root.getByText('Successfully cancelled task'));
//     });

//     it('cancel task button is disabled for user without required permission', () => {
//       const root = render(<TaskPanel tasks={[makeTaskWithPhases('task1', 1, 1)]} />, {
//         profile: { user: { username: 'test2', is_admin: false, roles: [] }, permissions: [] },
//       });
//       userEvent.click(root.getByText('task1'));
//       const button = root.getByLabelText('Cancel Task');
//       expect(button).toHaveClass('Mui-disabled');
//     });

//     it('cancel task button is disabled for completed task', () => {
//       const task = makeTaskWithPhases('task1', 1, 1);
//       task.summary.state = RmfTaskSummary.STATE_COMPLETED;
//       const root = render(<TaskPanel tasks={[task]} />);
//       userEvent.click(root.getByText('task1'));
//       const button = root.getByLabelText('Cancel Task');
//       expect(button).toHaveClass('Mui-disabled');
//     });

//     it('cancel task button is disabled for failed task', () => {
//       const task = makeTaskWithPhases('task1', 1, 1);
//       task.summary.state = RmfTaskSummary.STATE_FAILED;
//       const root = render(<TaskPanel tasks={[task]} />);
//       userEvent.click(root.getByText('task1'));
//       const button = root.getByLabelText('Cancel Task');
//       expect(button).toHaveClass('Mui-disabled');
//     });

//     it('cancel task button is disabled for cancelled task', () => {
//       const task = makeTaskWithPhases('task1', 1, 1);
//       task.summary.state = RmfTaskSummary.STATE_CANCELED;
//       const root = render(<TaskPanel tasks={[task]} />);
//       userEvent.click(root.getByText('task1'));
//       const button = root.getByLabelText('Cancel Task');
//       expect(button).toHaveClass('Mui-disabled');
//     });
//   });

//   describe('create task', () => {
//     it('clicking on create task button opens the create task form', () => {
//       const root = render(<TaskPanel tasks={[]} />);
//       userEvent.click(root.getByLabelText('Create Task'));
//       root.getByText('Create Task');
//     });

//     it('closing the create task form reset its states', () => {
//       const root = render(<TaskPanel tasks={[]} />);
//       userEvent.click(root.getByLabelText('Create Task'));
//       let priorityEl = root.getByLabelText('Priority') as HTMLInputElement;
//       userEvent.clear(priorityEl);
//       userEvent.type(priorityEl, '2');
//       expect(priorityEl.value).toBe('2');

//       userEvent.click(root.getByText('Cancel'));
//       userEvent.click(root.getByLabelText('Create Task'));
//       priorityEl = root.getByLabelText('Priority') as HTMLInputElement;
//       expect(priorityEl.value).toBe('0');
//     });
//   });

//   it('success snackbar is shown when successfully created a task', async () => {
//     const spy = jest.fn(() => Promise.resolve(undefined));
//     const root = render(<TaskPanel tasks={[]} submitTasks={spy} />);
//     userEvent.click(root.getByLabelText('Create Task'));
//     userEvent.click(root.getByText('Submit'));
//     await waitFor(() => root.getByText('Successfully created task'));
//   });

//   it('failure snackbar is shown when failed to created a task', async () => {
//     const spy = jest.fn(() => Promise.reject('error!!'));
//     const root = render(<TaskPanel tasks={[]} submitTasks={spy} />);
//     userEvent.click(root.getByLabelText('Create Task'));
//     userEvent.click(root.getByText('Submit'));
//     await waitFor(() => root.getByText('Failed to create task', { exact: false }));
//   });

//   it('pagination is shown when pagination option is provided', () => {
//     const spy = jest.fn();
//     const root = render(
//       <TaskPanel
//         tasks={[makeTaskWithPhases('test', 1, 1)]}
//         paginationOptions={{
//           count: 1,
//           page: 0,
//           rowsPerPage: 10,
//           rowsPerPageOptions: [10],
//           onPageChange: spy,
//         }}
//       />,
//     );
//     // NOTE: mui v5 is using the unicode char '–', different from '-'!!
//     root.getByText('1–1 of 1');
//   });

//   it('clicking on auto refresh button toggles auto refresh', () => {
//     const spy = jest.fn();
//     const root = render(<TaskPanel tasks={[]} onAutoRefresh={spy} />);
//     userEvent.click(root.getByLabelText('Disable auto refresh'));
//     expect(spy).toHaveBeenCalledTimes(1);
//     expect(spy.mock.calls[0][0]).toBe(false);
//     userEvent.click(root.getByLabelText('Enable auto refresh'));
//     expect(spy).toHaveBeenCalledTimes(2);
//     expect(spy.mock.calls[1][0]).toBe(true);
//   });

//   it('clicking on refresh button triggers onRefresh', () => {
//     const spy = jest.fn();
//     const root = render(<TaskPanel tasks={[]} onRefresh={spy} />);
//     userEvent.click(root.getByLabelText('Refresh'));
//     expect(spy).toHaveBeenCalledTimes(1);
//   });
// });
