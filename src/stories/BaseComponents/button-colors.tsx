import React from 'react';
import { Typography } from '@material-ui/core';

interface ButtonColorsProps {
    state: string;
    style: React.CSSProperties;
}

const rootStyles: React.CSSProperties = {
    display: 'flex',
    justifyContent: 'space-between',
    padding: '0.5rem 0'
}

export default function ButtonColors(props: ButtonColorsProps): React.ReactElement {

    const { state, style } = props;

    return (
        <div style={rootStyles}>
            <Typography variant="body1">{state}</Typography>
            <Typography style={style} variant="button">{state}</Typography>
        </div>
    )
}