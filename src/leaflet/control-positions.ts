import * as L from 'leaflet';

import './control-positions.css';

export type ControlPosition = (
  L.ControlPosition | 'topcenter' | 'bottomcenter' | 'leftcenter' | 'rightcenter' | 'center'
);

export function extendControlPositions() {
  // Implement feature from this PR: https://github.com/Leaflet/Leaflet/pull/5554
  const origInitControlPos = (L.Map as any).prototype._initControlPos;
  L.Map.include({
    _initControlPos() {
      origInitControlPos.call(this);
      this._controlCorners['topcenter'] = L.DomUtil.create(
        'div',
        'leaflet-top leaflet-horizontal-center',
        this._controlContainer,
      );
      this._controlCorners['bottomcenter'] = L.DomUtil.create(
        'div',
        'leaflet-bottom leaflet-horizontal-center',
        this._controlContainer,
      );
      this._controlCorners['leftcenter'] = L.DomUtil.create(
        'div',
        'leaflet-left leaflet-vertical-center',
        this._controlContainer,
      );
      this._controlCorners['rightcenter'] = L.DomUtil.create(
        'div',
        'leaflet-right leaflet-vertical-center',
        this._controlContainer,
      );
      this._controlCorners['center'] = L.DomUtil.create(
        'div',
        'leaflet-horizontal-center leaflet-vertical-center',
        this._controlContainer,
      );
    },
  });
}