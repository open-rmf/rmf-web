import React from 'react';
import ReactDOM from 'react-dom';
import MainMenu, { ItemState } from '../main-menu';

const mockItemState: ItemState = {
  doors: {},
  dispensers: {},
  lifts: {},
  robots: {},
};

it('renders without crashing', () => {
  const div = document.createElement('div');
  ReactDOM.render(
    <MainMenu
      pushView={jest.fn()}
      itemState={mockItemState}
      tasks={[]}
      notifications={[]}
      deletedNotifications={[]}
    />,
    div,
  );
  ReactDOM.unmountComponentAtNode(div);
});
