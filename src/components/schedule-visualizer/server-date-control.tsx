import React from 'react'
import _Control from 'react-leaflet-control'
import { MapControlProps, withLeaflet } from 'react-leaflet'
import styled from 'styled-components'

const Control = styled(_Control)`
  float: left;
  background: #fff;
  border-radius: 5px 5px 5px 5px;
  background-clip: padding-box;
  border: 2px solid rgba(0, 0, 0, 0.2);
  height: 44px;
  vertical-align: center;
`

const ServerDate = styled.div`
  padding: 3px 5px 0 5px;
  font-size: 2em;
`

export interface Props extends MapControlProps {
  date: Date
}

function ServerDateControl({ date }: Props) {
  return (
    <Control>
      <ServerDate title="Server Time">
        { date.toLocaleString() }
      </ServerDate>
    </Control>
  )
}

export default withLeaflet(ServerDateControl)