import Button from '@material-ui/core/Button';
import React from 'react';
import OmniPanel from '../omni-panel';
import OmniPanelView from '../omni-panel-view';

export default {
  title: 'Omni Panel',
};

export function SimplePanel() {
  const [view, setView] = React.useState(0);

  return (
    <OmniPanel view={view} style={{ width: 500, height: 500, border: '1px solid black' }}>
      <OmniPanelView id={0}>
        <Button variant="outlined" onClick={() => setView(1)}>
          Panel A
        </Button>
        <Button variant="outlined" onClick={() => setView(2)}>
          Panel B
        </Button>
        <Button variant="outlined" onClick={() => setView(3)}>
          Panel C
        </Button>
      </OmniPanelView>
      <OmniPanelView id={1}>
        <Button variant="outlined" onClick={() => setView(0)}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView id={2}>
        <Button variant="outlined" onClick={() => setView(0)}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView id={3}>
        <Button variant="outlined" onClick={() => setView(0)}>
          Back
        </Button>
      </OmniPanelView>
    </OmniPanel>
  );
}
