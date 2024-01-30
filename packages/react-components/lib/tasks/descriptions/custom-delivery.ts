interface GoToPlaceActivity {
  category: string;
  description: string;
}

interface LotPickupActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {
      cart_id: string;
      pickup_lot: string;
    };
  };
}

interface ZonePickupActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {
      cart_id: string;
      pickup_zone: string;
    };
  };
}

interface CartPickupPhase {
  activity: {
    category: string;
    description: {
      activities: [
        go_to_pickup: GoToPlaceActivity,
        pickup_cart: LotPickupActivity | ZonePickupActivity,
      ];
    };
  };
}

interface OneOfWaypoint {
  waypoint: string;
}

interface GoToOneOfThePlacesActivity {
  category: string;
  description: {
    one_of: OneOfWaypoint[];
    constraints: [
      {
        category: string;
        description: string;
      },
    ];
  };
}

interface DropoffActivity {
  category: string;
  description: {
    unix_millis_action_duration_estimate: number;
    category: string;
    description: {};
  };
}

interface OnCancelDropoff {
  category: string;
  description: [
    // dropoff_if_carrying_payload: DropoffActivity,
    go_to_one_of_the_places: GoToOneOfThePlacesActivity,
    on_cancel_dropoff: DropoffActivity,
  ];
}

interface DeliveryWithCancellationPhase {
  activity: {
    category: string;
    description: {
      activities: [go_to_place: GoToPlaceActivity];
    };
  };
  on_cancel: OnCancelDropoff[];
}

interface CartDropoffPhase {
  activity: {
    category: string;
    description: {
      activities: [delivery_dropoff: DropoffActivity];
    };
  };
}

export interface CustomDeliveryTaskDescription {
  category: string;
  phases: [
    pickup_phase: CartPickupPhase,
    delivery_phase: DeliveryWithCancellationPhase,
    dropoff_phase: CartDropoffPhase,
  ];
}

function makeGoToPlaceActivity(place: string): GoToPlaceActivity {
  return {
    category: 'go_to_place',
    description: place,
  };
}

function makeGoToOneOfThePlacesActivity(places: string[]): GoToOneOfThePlacesActivity {
  const waypoints: OneOfWaypoint[] = places.map((place) => {
    return { waypoint: place };
  });
  return {
    category: 'go_to_place',
    description: {
      one_of: waypoints,
      constraints: [
        {
          category: 'prefer_same_map',
          description: '',
        },
      ],
    },
  };
}

function makeDeliveryPickupActivity(
  delivery_category: string,
  cart_id: string,
  pickup_lot_or_zone: string,
): LotPickupActivity | ZonePickupActivity {
  return delivery_category === 'delivery_pickup'
    ? {
        category: 'perform_action',
        description: {
          unix_millis_action_duration_estimate: 60000,
          category: delivery_category,
          description: {
            cart_id: cart_id,
            pickup_lot: pickup_lot_or_zone,
          },
        },
      }
    : {
        category: 'perform_action',
        description: {
          unix_millis_action_duration_estimate: 60000,
          category: delivery_category,
          description: {
            cart_id: cart_id,
            pickup_zone: pickup_lot_or_zone,
          },
        },
      };
}

function makeDeliveryDropoffActivity(): DropoffActivity {
  return {
    category: 'perform_action',
    description: {
      unix_millis_action_duration_estimate: 60000,
      category: 'delivery_dropoff',
      description: {},
    },
  };
}

export function makeCustomDeliveryTaskDescription(
  delivery_category: string,
  pickup_place: string,
  cart_id: string,
  pickup_lot_or_zone: string,
  dropoff_place: string,
  on_cancel_dropoff_places: string[],
): CustomDeliveryTaskDescription {
  return {
    category: delivery_category,
    phases: [
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [
              makeGoToPlaceActivity(pickup_place),
              makeDeliveryPickupActivity(delivery_category, cart_id, pickup_lot_or_zone),
            ],
          },
        },
      },
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [makeGoToPlaceActivity(dropoff_place)],
          },
        },
        on_cancel: [
          {
            category: 'sequence',
            description: [
              makeGoToOneOfThePlacesActivity(on_cancel_dropoff_places),
              makeDeliveryDropoffActivity(),
            ],
          },
        ],
      },
      {
        activity: {
          category: 'sequence',
          description: {
            activities: [makeDeliveryDropoffActivity()],
          },
        },
      },
    ],
  };
}

export function makeDefaultCustomDeliveryTaskDescription(): CustomDeliveryTaskDescription {
  return makeCustomDeliveryTaskDescription('delivery_pickup', '', '', '', '', []);
}
