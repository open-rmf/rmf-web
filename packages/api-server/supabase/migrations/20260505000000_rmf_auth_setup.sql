-- Seed/bootstrap SQL for the rmf-web Supabase project.
--
-- This file is idempotent. Apply with `supabase db reset` (local) or
-- `supabase db seed` after `supabase db push`.

-- ---------------------------------------------------------------------------
-- 1. Storage for per-user role assignments.
-- ---------------------------------------------------------------------------
-- Supabase auth.users is owned by the auth schema and shouldn't be touched
-- directly. Keep our own roles table keyed on the auth user id.
create table if not exists public.rmf_user_roles (
  user_id uuid primary key references auth.users(id) on delete cascade,
  roles   text[] not null default '{}'::text[]
);

alter table public.rmf_user_roles enable row level security;

-- Only service-role keys can read/write. The api-server reads roles via JWT
-- claims (populated by the access-token hook below), not by querying this
-- table directly, so end-users never need access.
drop policy if exists "service role only" on public.rmf_user_roles;
create policy "service role only"
  on public.rmf_user_roles
  for all
  to service_role
  using (true)
  with check (true);

-- ---------------------------------------------------------------------------
-- 2. Custom access-token hook.
-- ---------------------------------------------------------------------------
-- Supabase calls this on every JWT mint and merges the returned `claims`
-- back into the token. We inject `app_metadata.roles` so the api-server can
-- recognise admins without a round-trip to the DB.
--
-- Wired in supabase/config.toml under [auth.hook.custom_access_token].
create or replace function public.rmf_access_token_hook(event jsonb)
returns jsonb
language plpgsql
stable
as $$
declare
  claims jsonb;
  user_roles text[];
  app_meta jsonb;
begin
  claims := event->'claims';

  select coalesce(roles, '{}'::text[])
    into user_roles
    from public.rmf_user_roles
   where user_id = (event->>'user_id')::uuid;

  app_meta := coalesce(claims->'app_metadata', '{}'::jsonb)
              || jsonb_build_object('roles', coalesce(to_jsonb(user_roles), '[]'::jsonb));

  claims := jsonb_set(claims, '{app_metadata}', app_meta, true);
  return jsonb_build_object('claims', claims);
end;
$$;

grant execute on function public.rmf_access_token_hook(jsonb) to supabase_auth_admin;
revoke execute on function public.rmf_access_token_hook(jsonb) from authenticated, anon, public;

-- ---------------------------------------------------------------------------
-- 3. Convenience helper to grant admin to a user by email.
-- ---------------------------------------------------------------------------
-- Run from the SQL editor or `supabase db execute` to bootstrap the first
-- admin after the user has signed up:
--   select public.rmf_grant_superuser('admin@example.com');
create or replace function public.rmf_grant_superuser(p_email text)
returns void
language plpgsql
security definer
set search_path = public
as $$
declare
  uid uuid;
begin
  select id into uid from auth.users where email = p_email;
  if uid is null then
    raise exception 'no user with email %', p_email;
  end if;

  insert into public.rmf_user_roles (user_id, roles)
       values (uid, array['superuser'])
  on conflict (user_id)
       do update set roles = (
         select array(select distinct unnest(public.rmf_user_roles.roles || excluded.roles))
       );
end;
$$;
