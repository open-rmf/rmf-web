import Button from '@material-ui/core/Button';
import React from 'react';
import OmniPanel from '../lib/omni-panel';
import OmniPanelView from '../lib/omni-panel-view';
import StackNavigator from '../lib/stack-navigator';
export default function SimpleOmniPanel() {
  var _a = React.useState(0),
    view = _a[0],
    setView = _a[1];
  var stack = React.useMemo(function () {
    return new StackNavigator(0);
  }, []);
  return React.createElement(
    OmniPanel,
    {
      view: view,
      style: {
        width: 500,
        height: 200,
        border: '1px solid black',
        borderTopLeftRadius: 16,
        borderTopRightRadius: 16,
      },
      onBack: function () {
        return setView(stack.pop());
      },
      onHome: function () {
        return setView(stack.reset());
      },
    },
    React.createElement(
      OmniPanelView,
      { id: 0 },
      React.createElement(
        Button,
        {
          variant: 'outlined',
          onClick: function () {
            return stack.push(1), setView(1);
          },
        },
        'Panel A',
      ),
      React.createElement(
        Button,
        {
          variant: 'outlined',
          onClick: function () {
            return stack.push(2), setView(2);
          },
        },
        'Panel B',
      ),
      React.createElement(
        Button,
        {
          variant: 'outlined',
          onClick: function () {
            return stack.push(3), setView(3);
          },
        },
        'Panel C',
      ),
    ),
    React.createElement(
      OmniPanelView,
      { id: 1 },
      React.createElement(
        Button,
        {
          variant: 'outlined',
          onClick: function () {
            return setView(stack.pop());
          },
        },
        'Back',
      ),
    ),
    React.createElement(
      OmniPanelView,
      { id: 2 },
      React.createElement(
        Button,
        {
          variant: 'outlined',
          onClick: function () {
            return setView(stack.pop());
          },
        },
        'Back',
      ),
    ),
    React.createElement(
      OmniPanelView,
      { id: 3 },
      React.createElement(
        Button,
        {
          variant: 'outlined',
          onClick: function () {
            return setView(stack.pop());
          },
        },
        'Back',
      ),
    ),
  );
}
