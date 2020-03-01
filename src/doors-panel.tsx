import React from 'react';
import * as RomiCore from '@osrf/romi-js-core-interfaces';

import {
	ExpansionPanel,
	ExpansionPanelSummary,
	ExpansionPanelDetails,
	Typography,
	makeStyles,
} from '@material-ui/core';

import {
	ExpandMore as ExpandMoreIcon,
} from '@material-ui/icons';
import { CSSProperties } from '@material-ui/core/styles/withStyles';

function doorModeToString(doorMode: RomiCore.DoorMode): string {
	switch (doorMode.value) {
		case RomiCore.DoorMode.MODE_OPEN:
			return 'OPEN';
		case RomiCore.DoorMode.MODE_CLOSED:
			return 'CLOSED';
		case RomiCore.DoorMode.MODE_MOVING:
			return 'MOVING';
		default:
			return 'UNKNOWN';
	}
}

const useStyles = makeStyles(() => ({
	expansionSummaryContent: {
		alignItems: 'center',
	}
}));

const useDoorModeLabelStyles = makeStyles(theme => {
	const base: CSSProperties = {
		marginLeft: 50,
		borderRadius: theme.shape.borderRadius,
		borderStyle: 'solid',
		border: 2,
		padding: 5,
		width: theme.typography.fontSize * 7,
		textAlign: 'center',
	};

	return {
		open: {
			...base,
			borderColor: theme.palette.success.main,
		},

		closed: {
			...base,
			borderColor: theme.palette.error.main,
		},

		moving: {
			...base,
			borderColor: theme.palette.warning.main,
		},
}});

interface DoorsPanelProps {
	doors: Map<RomiCore.Door, RomiCore.DoorState>;
}

export default function DoorsPanel(props: DoorsPanelProps): JSX.Element {
	const classes = useStyles();

	const doorModeLabelClasses = useDoorModeLabelStyles();
	const doorModelLabelClass = (doorMode: RomiCore.DoorMode) => {
		switch (doorMode.value) {
			case RomiCore.DoorMode.MODE_OPEN:
				return doorModeLabelClasses.open;
			case RomiCore.DoorMode.MODE_CLOSED:
				return doorModeLabelClasses.closed;
			case RomiCore.DoorMode.MODE_MOVING:
				return doorModeLabelClasses.moving;
			default:
				return '';
		}
	}

	const listItems = Array.from(props.doors.entries()).map(([door, doorState]) => (
		<ExpansionPanel key={door.name}>
			<ExpansionPanelSummary classes={{content: classes.expansionSummaryContent}} expandIcon={<ExpandMoreIcon />}>
				<Typography variant="h5">{door.name}</Typography>
				<div className={doorModelLabelClass(doorState.current_mode)}>
					<Typography variant="button">{doorModeToString(doorState.current_mode)}</Typography>
				</div>
			</ExpansionPanelSummary>
			<ExpansionPanelDetails>
				<Typography variant="body1">
					Type: {door.door_type}
					<br />
					Motion Direction: {door.motion_direction}
					<br />
					Motion Range: {door.motion_range}
					<br />
					Location: {door.v1_x}, {door.v1_y}
					<br />
				</Typography>
			</ExpansionPanelDetails>
		</ExpansionPanel>
	));

	return (
		<div>
			{listItems}
		</div>
	)
}
