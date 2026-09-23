"""Combine short geometry with longer, accumulated displacement evidence."""
def combine(short, long):
    result = dict(short)
    result['windows'] = {'short': short, 'long': long}
    # Current local geometry controls whether a long-baseline contradiction can
    # be trusted. Old room features must not override a newly ambiguous hallway.
    if (short['state'] in ('CONSISTENT', 'MOTION_CONTRADICTED') and long is not None
            and long['state'] == 'MOTION_CONTRADICTED'):
        result.update(state='MOTION_CONTRADICTED',
                      reason='accumulated scan evidence rejects drive translation',
                      evidence_window='long',
                      near_zero_supported=long.get('near_zero_supported', False),
                      motion_kind=long.get('motion_kind', 'DISAGREEMENT'))
    else:
        result['evidence_window'] = 'short'
    return result
