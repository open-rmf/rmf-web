-- Fix: the access-token hook runs as supabase_auth_admin, which our
-- "service role only" RLS policy on rmf_user_roles excludes. Recreate
-- the function as security definer so it bypasses RLS entirely, and
-- grant supabase_auth_admin direct access to the table as a belt-and-
-- braces measure (used by the dashboard's auth admin UI too).

create or replace function public.rmf_access_token_hook(event jsonb)
returns jsonb
language plpgsql
stable
security definer
set search_path = public
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

grant usage on schema public to supabase_auth_admin;
grant select on public.rmf_user_roles to supabase_auth_admin;

drop policy if exists "auth admin read" on public.rmf_user_roles;
create policy "auth admin read"
  on public.rmf_user_roles
  for select
  to supabase_auth_admin
  using (true);
