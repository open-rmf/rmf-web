import { fireEvent, render, screen } from '@testing-library/react';
import { TaskFavorite } from 'api-client';
import { beforeEach, describe, expect, it, vi } from 'vitest';

import { LocalizationProvider } from './../locale';
import { DaySelectorSwitch, FavoriteTask, getDefaultTaskRequest, TaskForm } from './task-form';
import { PatrolTaskDefinition } from './types/patrol';
import { getDefaultTaskDescription, getTaskRequestCategory } from './types/utils';

const mockUser = 'mock_user';
const mockFleets = {
  fleet_1: ['robot_1'],
  fleet_2: ['robot_2', 'robot_3'],
};
const mockCleanZones = ['clean_zone_1', 'clean_zone_2'];
const mockWaypoints = ['waypoint_1', 'waypoint_2', 'waypoint_3'];
const mockPickupZones = ['pickup_zone_1', 'pickup_zone_2'];
const mockCartIds = ['cart_1', 'cart_2', 'cart_3'];
const mockPickupPoints = {
  pickup_1: 'handler_1',
  pickup_2: 'handler_2',
};
const mockDropoffPoints = {
  dropoff_1: 'handler_3',
  dropoff_2: 'handler_4',
};

const onDispatchTask = vi.fn();
const onScheduleTask = vi.fn();
const onEditScheduleTask = vi.fn();
const onSuccess = vi.fn();
const onFail = vi.fn();
const onSuccessFavoriteTask = vi.fn();
const onFailFavoriteTask = vi.fn();
const submitFavoriteTask = vi.fn();
const deleteFavoriteTask = vi.fn();
const onSuccessScheduling = vi.fn();
const onFailScheduling = vi.fn();

describe('FavoriteTask', () => {
  const mockListItemClick = vi.fn();
  const mockSetFavoriteTask = vi.fn();
  const mockSetOpenDialog = vi.fn();
  const mockSetCallToDelete = vi.fn();
  const mockSetCallToUpdate = vi.fn();
  const mockFavoriteTask: TaskFavorite = {
    id: 'test-id',
    name: 'test-name',
    unix_millis_earliest_start_time: 0,
    priority: null,
    category: 'test-category',
    description: null,
    user: 'test-user',
    task_definition_id: 'test-definition-id',
  };

  beforeEach(() => {
    mockListItemClick.mockClear();
    mockSetFavoriteTask.mockClear();
    mockSetOpenDialog.mockClear();
    mockSetCallToDelete.mockClear();
    mockSetCallToUpdate.mockClear();
  });

  it('renders without crashing', () => {
    render(
      <FavoriteTask
        listItemText="Test Task"
        listItemClick={mockListItemClick}
        favoriteTask={mockFavoriteTask}
        setFavoriteTask={mockSetFavoriteTask}
        setOpenDialog={mockSetOpenDialog}
        setCallToDelete={mockSetCallToDelete}
        setCallToUpdate={mockSetCallToUpdate}
      />,
    );
    expect(screen.getByText('Test Task')).toBeTruthy();
  });

  it('calls listItemClick and setCallToUpdate(false) when the list item is clicked', () => {
    render(
      <FavoriteTask
        listItemText="Test Task"
        listItemClick={mockListItemClick}
        favoriteTask={mockFavoriteTask}
        setFavoriteTask={mockSetFavoriteTask}
        setOpenDialog={mockSetOpenDialog}
        setCallToDelete={mockSetCallToDelete}
        setCallToUpdate={mockSetCallToUpdate}
      />,
    );
    const listItem = screen.getByTestId('listitem-button');
    fireEvent.click(listItem);
    expect(mockListItemClick).toHaveBeenCalled();
    expect(mockSetCallToUpdate).toHaveBeenCalledWith(false);
  });

  it('calls listItemClick and setCallToUpdate(true) when the update icon is clicked', () => {
    render(
      <FavoriteTask
        listItemText="Test Task"
        listItemClick={mockListItemClick}
        favoriteTask={mockFavoriteTask}
        setFavoriteTask={mockSetFavoriteTask}
        setOpenDialog={mockSetOpenDialog}
        setCallToDelete={mockSetCallToDelete}
        setCallToUpdate={mockSetCallToUpdate}
      />,
    );
    const updateIcon = screen.getByLabelText('update');
    fireEvent.click(updateIcon);
    expect(mockListItemClick).toHaveBeenCalled();
    expect(mockSetCallToUpdate).toHaveBeenCalledWith(true);
  });

  it('calls setOpenDialog, setFavoriteTask, and setCallToDelete when the delete icon is clicked', () => {
    render(
      <FavoriteTask
        listItemText="Test Task"
        listItemClick={mockListItemClick}
        favoriteTask={mockFavoriteTask}
        setFavoriteTask={mockSetFavoriteTask}
        setOpenDialog={mockSetOpenDialog}
        setCallToDelete={mockSetCallToDelete}
        setCallToUpdate={mockSetCallToUpdate}
      />,
    );
    const deleteIcon = screen.getByLabelText('delete');
    fireEvent.click(deleteIcon);
    expect(mockSetOpenDialog).toHaveBeenCalledWith(true);
    expect(mockSetFavoriteTask).toHaveBeenCalledWith(mockFavoriteTask);
    expect(mockSetCallToDelete).toHaveBeenCalledWith(true);
  });
});

describe('getDefaultTaskRequest', () => {
  it('invalid task definition id', () => {
    const request = getDefaultTaskRequest('invalid');
    expect(request).toBeNull();
  });

  it('patrol task definition id', () => {
    const request = getDefaultTaskRequest(PatrolTaskDefinition.taskDefinitionId);
    expect(request?.category).toBe(getTaskRequestCategory(PatrolTaskDefinition.taskDefinitionId));
    expect(request?.description).toStrictEqual(
      getDefaultTaskDescription(PatrolTaskDefinition.taskDefinitionId),
    );
  });
});

describe('DaySelectorSwitch', () => {
  const mockOnChange = vi.fn();

  beforeEach(() => {
    mockOnChange.mockClear();
  });

  it('renders without crashing', () => {
    render(
      <DaySelectorSwitch
        disabled={false}
        onChange={mockOnChange}
        value={[true, true, true, true, true, true, true]}
      />,
    );
    expect(screen.getByText('Mon')).toBeTruthy();
    expect(screen.getByText('Tue')).toBeTruthy();
    expect(screen.getByText('Wed')).toBeTruthy();
    expect(screen.getByText('Thu')).toBeTruthy();
    expect(screen.getByText('Fri')).toBeTruthy();
    expect(screen.getByText('Sat')).toBeTruthy();
    expect(screen.getByText('Sun')).toBeTruthy();
  });

  it('onChange triggered', () => {
    render(
      <DaySelectorSwitch
        disabled={false}
        onChange={mockOnChange}
        value={[true, true, true, true, true, true, true]}
      />,
    );

    const monChip = screen.getByTestId('Mon');
    fireEvent.click(monChip);
    expect(mockOnChange).toHaveBeenCalledWith([false, true, true, true, true, true, true]);
    const tueChip = screen.getByTestId('Tue');
    fireEvent.click(tueChip);
    expect(mockOnChange).toHaveBeenCalledWith([false, false, true, true, true, true, true]);

    fireEvent.click(monChip);
    expect(mockOnChange).toHaveBeenCalledWith([true, false, true, true, true, true, true]);
  });
});

describe('Task form', () => {
  it('Task form renders', async () => {
    render(
      <LocalizationProvider>
        <TaskForm
          user={mockUser}
          fleets={mockFleets}
          // taskToDisplay
          cleaningZones={mockCleanZones}
          patrolWaypoints={mockWaypoints}
          pickupZones={mockPickupZones}
          cartIds={mockCartIds}
          pickupPoints={mockPickupPoints}
          dropoffPoints={mockDropoffPoints}
          favoritesTasks={[]}
          // schedule={}
          // taskRequest={}
          onDispatchTask={onDispatchTask}
          onScheduleTask={onScheduleTask}
          onEditScheduleTask={onEditScheduleTask}
          onSuccess={onSuccess}
          onFail={onFail}
          onSuccessFavoriteTask={onSuccessFavoriteTask}
          onFailFavoriteTask={onFailFavoriteTask}
          submitFavoriteTask={submitFavoriteTask}
          deleteFavoriteTask={deleteFavoriteTask}
          onSuccessScheduling={onSuccessScheduling}
          onFailScheduling={onFailScheduling}
          open={true}
        />
      </LocalizationProvider>,
    );
  });
});
