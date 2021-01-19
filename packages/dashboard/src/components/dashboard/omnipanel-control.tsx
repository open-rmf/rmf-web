import DashboardIcon from '@material-ui/icons/Dashboard';
import Leaflet from 'leaflet';
import React from 'react';
import ReactDOM from 'react-dom';
import { MapControl, MapControlProps, withLeaflet } from 'react-leaflet';
import { ReducerDashboardDispatch } from './reducers/dashboard-reducer';

export interface OmniPanelControlProps extends MapControlProps {
  show: boolean;
  dashboardDispatch: ReducerDashboardDispatch;
}

class OmniPanelControl extends MapControl<OmniPanelControlProps> {
  constructor(props: OmniPanelControlProps) {
    super(props);
    this._container = Leaflet.DomUtil.create('div');
    this._container.className = 'leaflet-bar';
  }

  createLeafletElement(): Leaflet.Control {
    const LeafletControl = Leaflet.Control.extend({
      onAdd: () => {
        return this._container;
      },
    });
    return new LeafletControl();
  }

  updateLeafletElement(fromProps: OmniPanelControlProps, toProps: OmniPanelControlProps) {
    super.updateLeafletElement(fromProps, toProps);
    const { show, dashboardDispatch } = toProps;
    if (!show) {
      this._container.setAttribute('style', 'display: none');
    } else {
      this._container.setAttribute('style', '');
    }
    ReactDOM.render(
      // leaflet css uses `a` element to style controls
      // eslint-disable-next-line jsx-a11y/anchor-is-valid
      <a href="#" onClick={() => dashboardDispatch.setShowOmniPanel(true)}>
        <DashboardIcon style={{ verticalAlign: 'middle' }} />
      </a>,
      this._container,
    );
  }

  private _container: HTMLElement;
}

export default withLeaflet(OmniPanelControl);

// /**
//  * Mimics a leaflet control. We could make this an actual leaflet control but it is easier to
//  * make a react component, the downside is that leaflet will not see this fake control
//  * and will not be able to stack it so that it doesn't overlap with other controls.
//  */
// export function OmniPanelControl(): JSX.Element {
//   const classes = useStyles();

//   return (
//     <div className={`leaflet-control-layers leaflet-control ${classes.omniPanelControl}`}>
//       <DashboardIcon />
//     </div>
//   );
// }
