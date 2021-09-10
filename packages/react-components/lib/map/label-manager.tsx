import { useTheme } from '@material-ui/core';
import EventEmitter from 'eventemitter3';
import L from 'leaflet';
import React from 'react';
import { uniqueId } from '../utils';
import { NameLabel, NameLabelProps } from './label-marker';

interface OverrideLabelProps {
  anchorX: number;
  anchorY: number;
  arrowLength: number;
  radius: number;
  theta: number;
  transform: string;
}

export interface LabelTarget {
  centerX: number;
  centerY: number;
  radius: number;
}

interface ManagedLabelEvents {
  change: OverrideLabelProps;
}

interface ManagedLabel {
  target: LabelTarget;
  managedProps: OverrideLabelProps;
  events: EventEmitter<ManagedLabelEvents>;
}

export class LabelManager {
  lmap: L.Map;

  constructor(lmap: L.Map) {
    this.lmap = lmap;
    this.lmap.on('zoom', () => this._updateAllLabels());
  }

  addLabel(id: string, target: LabelTarget): ManagedLabel {
    const managedProps = this._getManagedProps(target);
    const label = {
      target,
      managedProps,
      events: new EventEmitter<ManagedLabelEvents>(),
    };
    this._labels[id] = label;
    return label;
  }

  removeLabel(id: string): void {
    delete this._labels[id];
  }

  private _labels: Record<string, ManagedLabel> = {};

  private _getManagedProps(target: LabelTarget): OverrideLabelProps {
    // TODO: return non-overlapping labels.
    const thetaRad = (-30 * Math.PI) / 180;
    return {
      anchorX: target.centerX + Math.cos(thetaRad) * target.radius,
      anchorY: target.centerY + Math.sin(thetaRad) * target.radius,
      arrowLength: 1,
      radius: 0.2,
      theta: -45,
      transform: `scale(${20 / 2 ** this.lmap.getZoom()})`,
    };
  }

  private _updateAllLabels(): void {
    Object.values(this._labels).forEach((label) => {
      label.managedProps = this._getManagedProps(label.target);
      label.events.emit('change', label.managedProps);
    });
  }
}

export const LabelManagerContext = React.createContext<LabelManager | null>(null);

export type ManagedNameLabelProps = Omit<
  NameLabelProps,
  keyof OverrideLabelProps | 'fontSize' | 'stroke' | 'strokeWidth'
> & {
  labelTarget: LabelTarget;
};

export function ManagedNameLabel({
  labelTarget,
  ...otherProps
}: ManagedNameLabelProps): JSX.Element | null {
  const theme = useTheme();
  const labelManager = React.useContext(LabelManagerContext);
  const [managedProps, setManagedProps] = React.useState<OverrideLabelProps | null>(null);

  React.useEffect(() => {
    if (!labelManager) {
      console.error(
        'ManagedNameLabel: No LabelManager found, make sure that this component is a descendent of Map',
      );
      return;
    }
    const labelId = `label-${uniqueId()}`;
    const newLabel = labelManager.addLabel(labelId, labelTarget);
    setManagedProps(newLabel.managedProps);
    newLabel.events.on('change', setManagedProps);
    return () => {
      labelManager.removeLabel(labelId);
    };
  }, [labelManager, labelTarget]);

  return managedProps ? (
    <NameLabel
      {...managedProps}
      fontSize={1}
      stroke={theme.palette.primary.main}
      strokeWidth={0.1}
      {...otherProps}
    />
  ) : null;
}
